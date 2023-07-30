/****************************************************************************
 *
 *   Copyright (c) 2012-2013, 2017 PX4 Development Team. All rights reserved.
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
 * @file filter.h
 * Filter functions declarations
 *
 * @author volvox <volvox@stalya.com>
 *
 */

#pragma once

#include <math.h>

namespace filter {

/**
 ** class Filter
 */
class __EXPORT Filter
{
public:
	typedef enum {
		FILTER_DISABLE,
		FILTER_MEDIAN,
		FILTER_LINREG,
	} filter_type_t;

	Filter(int type = FILTER_MEDIAN, int loop = 1);
	~Filter();

	float insert(int loop, float data);

private:
	static const int REGRES_DATALEN = 50;
	static const int MEDIAN_DATALEN = 15;
	static const int MEDIAN_INDEX 	= MEDIAN_DATALEN/2;

	typedef struct __attribute__((__packed__))  median_filter {
		float buffer[MEDIAN_DATALEN];
		uint64_t index;
	} median_filter_t;

	typedef struct __attribute__((__packed__))  regres_data {
		float x;
		float y;
	} regres_data_t;

	typedef struct __attribute__((__packed__)) regres_filter {
		regres_data_t buffer[REGRES_DATALEN];
		uint64_t index;
	} regres_filter_t;

	static float InvSqrt(float x);
	static int Comparator(const void *v1, const void * v2);

	float MedianInsert(int loop, float data);
	void LinRegInsert(int loop, float x, float y);
	int LinRegress(regres_data_t *xy, int n, float *a, float *b, float *r);

	int _type{0};
	int _loop{0};

	median_filter_t * _median{nullptr};
	regres_filter_t * _regres{nullptr};
};

}
