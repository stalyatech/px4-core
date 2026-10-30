/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Technology. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file TiltHeightEkf.hpp
 *
 * Extended Kalman filter of the radar/IMU tilt rig. Three layers share one
 * filter with the state x = [tilt, gyro bias, axis height]:
 *
 *  1. Gyro prediction with an accelerometer tilt update.
 *  2. Altimeter slant range R = (h - xa sin(t)) / cos(t) + za
 *  3. Ground echo of the obstacle radar r = (h - xm sin(t) + zm cos(t)) / sin(t + b)
 *     where b is the half elevation beam. Radar objects inside a gate scaled
 *     by the estimate uncertainty are ground echoes, not obstacles.
 *
 * Tilt is positive nose down. (xa, za) and (xm, zm) are the antenna faces
 * relative to the tilt axis at zero tilt.
 */

#pragma once

#include <math.h>
#include <matrix/math.hpp>

class TiltHeightEkf
{
public:
	using Vector3f = matrix::Vector3f;
	using Matrix3f = matrix::SquareMatrix3f;

	enum State { TILT = 0, BIAS = 1, HEIGHT = 2 };

	struct Config {
		float gyro_noise{1e-3f};         // [rad/s/sqrt(Hz)]
		float bias_random_walk{1e-5f};   // [rad/s^2/sqrt(Hz)]
		float height_random_walk{2e-3f}; // [m/s/sqrt(Hz)]
		float alt_noise{0.01f};          // [m]
		float ground_noise{0.10f};       // [m]
		float alt_gate{4.f};             // [sigma]
		float ground_gate{3.f};          // [sigma]
		float half_beam{0.1222f};        // [rad] obstacle radar half elevation beam
		float alt_face_x{0.f};           // [m]
		float alt_face_z{0.f};           // [m]
		float mr72_face_x{0.f};          // [m]
		float mr72_face_z{0.f};          // [m]
	};

	struct Innovation {
		float innovation{NAN};           // measurement minus prediction
		float variance{NAN};             // innovation variance
		float nis{NAN};                  // normalised innovation squared
		bool accepted{false};            // inside the gate
	};

	Config &config() { return _cfg; }

	void init(float tilt, float height)
	{
		_x = Vector3f(tilt, 0.f, height);
		_P.setZero();
		_P(TILT, TILT) = 0.1f * 0.1f;
		_P(BIAS, BIAS) = 0.02f * 0.02f;
		_P(HEIGHT, HEIGHT) = 0.2f * 0.2f;
	}

	/** @param rate measured nose down rate [rad/s], @param dt [s] */
	void predict(float rate, float dt)
	{
		_x(TILT) += (rate - _x(BIAS)) * dt;

		Matrix3f F;
		F.setIdentity();
		F(TILT, BIAS) = -dt;

		_P = F * _P * F.transpose();
		_P(TILT, TILT) += _cfg.gyro_noise * _cfg.gyro_noise * dt;
		_P(BIAS, BIAS) += _cfg.bias_random_walk * _cfg.bias_random_walk * dt;
		_P(HEIGHT, HEIGHT) += _cfg.height_random_walk * _cfg.height_random_walk * dt;
	}

	/** Layer 1: tilt from the gravity direction, @param noise [rad] */
	Innovation update_tilt(float angle, float noise)
	{
		return update(angle - _x(TILT), Vector3f(1.f, 0.f, 0.f), noise * noise, INFINITY);
	}

	/** Slant range the altimeter should read for the current estimate. */
	float altimeter_range() const
	{
		const float c = cosf(_x(TILT));
		return (_x(HEIGHT) - _cfg.alt_face_x * sinf(_x(TILT))) / c + _cfg.alt_face_z;
	}

	/** Axis height for a slant range at the current tilt. */
	float height_from_range(float range) const
	{
		return (range - _cfg.alt_face_z) * cosf(_x(TILT)) + _cfg.alt_face_x * sinf(_x(TILT));
	}

	/** Layer 2: altimeter range [m]. */
	Innovation update_altimeter(float range)
	{
		const float s = sinf(_x(TILT));
		const float c = cosf(_x(TILT));
		const Vector3f H((_x(HEIGHT) * s - _cfg.alt_face_x) / (c * c), 0.f, 1.f / c);

		return update(range - altimeter_range(), H, _cfg.alt_noise * _cfg.alt_noise, _cfg.alt_gate);
	}

	/** Forget the height after a jump the altimeter gate keeps rejecting. */
	void reset_height(float height)
	{
		_x(HEIGHT) = height;

		for (int i = 0; i < 3; i++) {
			_P(i, HEIGHT) = 0.f;
			_P(HEIGHT, i) = 0.f;
		}

		_P(HEIGHT, HEIGHT) = _cfg.alt_noise * _cfg.alt_noise;
	}

	/** True when the lower beam edge reaches the ground in front of the radar. */
	bool ground_visible() const
	{
		return (_x(TILT) + _cfg.half_beam) > GROUND_MIN_GRAZE && ground_face_height() > 0.f;
	}

	/** Expected range of the ground echo [m], valid if ground_visible(). */
	float ground_range() const
	{
		return ground_face_height() / sinf(_x(TILT) + _cfg.half_beam);
	}

	/** Gate half width around ground_range() [m]. */
	float ground_gate_width() const
	{
		const Vector3f H = ground_jacobian();
		return _cfg.ground_gate * sqrtf((H.transpose() * _P * H)(0, 0) + _cfg.ground_noise * _cfg.ground_noise);
	}

	/** Layer 3: is this radar object the ground echo? Does not change the state. */
	Innovation check_ground(float range) const
	{
		Innovation out{};

		if (ground_visible()) {
			const Vector3f H = ground_jacobian();
			out.innovation = range - ground_range();
			out.variance = (H.transpose() * _P * H)(0, 0) + _cfg.ground_noise * _cfg.ground_noise;
			out.nis = out.innovation * out.innovation / out.variance;
			out.accepted = out.nis <= _cfg.ground_gate * _cfg.ground_gate;
		}

		return out;
	}

	/** Layer 3: use a confirmed ground echo as a measurement of h and tilt. */
	Innovation update_ground(float range)
	{
		return update(range - ground_range(), ground_jacobian(), _cfg.ground_noise * _cfg.ground_noise, _cfg.ground_gate);
	}

	float tilt() const { return _x(TILT); }
	float bias() const { return _x(BIAS); }
	float height() const { return _x(HEIGHT); }
	float tilt_variance() const { return _P(TILT, TILT); }
	float bias_variance() const { return _P(BIAS, BIAS); }
	float height_variance() const { return _P(HEIGHT, HEIGHT); }

	/** Normalised estimation error squared against a known true state. */
	float nees(float tilt, float bias, float height) const
	{
		// In correlation form: the states differ by orders of magnitude and
		// the inverse of P itself is not reliable in single precision.
		const Vector3f e = Vector3f(tilt, bias, height) - _x;
		Vector3f e_n;
		Matrix3f C;

		for (int r = 0; r < 3; r++) {
			e_n(r) = e(r) / sqrtf(_P(r, r));

			for (int c = 0; c < 3; c++) {
				C(r, c) = _P(r, c) / sqrtf(_P(r, r) * _P(c, c));
			}
		}

		Matrix3f C_inv;

		if (!matrix::inv(C, C_inv)) {
			return NAN;
		}

		return (e_n.transpose() * C_inv * e_n)(0, 0);
	}

private:
	static constexpr float GROUND_MIN_GRAZE{0.0035f}; // [rad] 0.2 deg

	float ground_face_height() const
	{
		return _x(HEIGHT) - _cfg.mr72_face_x * sinf(_x(TILT)) + _cfg.mr72_face_z * cosf(_x(TILT));
	}

	Vector3f ground_jacobian() const
	{
		const float a = _x(TILT) + _cfg.half_beam;
		const float sa = sinf(a);
		const float z = ground_face_height();
		const float dz = -_cfg.mr72_face_x * cosf(_x(TILT)) - _cfg.mr72_face_z * sinf(_x(TILT));
		return Vector3f((dz * sa - z * cosf(a)) / (sa * sa), 0.f, 1.f / sa);
	}

	Innovation update(float innovation, const Vector3f &H, float R, float gate)
	{
		Innovation out{};
		const Vector3f PHt = _P * H;
		const float S = H.dot(PHt) + R;

		out.innovation = innovation;
		out.variance = S;
		out.nis = innovation * innovation / S;
		out.accepted = out.nis <= gate * gate;

		if (!out.accepted) {
			return out;
		}

		const Vector3f K = PHt / S;
		_x += K * innovation;

		// Joseph form keeps P symmetric and positive
		Matrix3f IKH;
		IKH.setIdentity();

		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				IKH(r, c) -= K(r) * H(c);
			}
		}

		_P = IKH * _P * IKH.transpose();

		for (int r = 0; r < 3; r++) {
			for (int c = 0; c < 3; c++) {
				_P(r, c) += K(r) * K(c) * R;
			}
		}

		return out;
	}

	Config _cfg{};
	Vector3f _x{};
	Matrix3f _P{};
};
