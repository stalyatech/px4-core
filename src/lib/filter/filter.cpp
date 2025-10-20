/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file filter.c
 * Filter functions
 *
 * @author volvox <volvox@stalya.com>
 *
 */

#include <stdio.h>
#include "filter.h"

#include <px4_platform_common/defines.h>

using namespace filter;

#define FLOAT_ZERO	(1e-9f)

/*******************************************************************************
* Function Name  : Constructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
Filter::Filter(int type, int loop) :
	_type(type),
	_loop(loop)
{
	/* bound for minimum loop count */
	if (_loop < 1) {
		_loop = 1;
	}

	/* check the filter type */
	switch (_type) {
		case FILTER_MEDIAN: {
			_median = new median_filter_t[_loop];
			if (_median != nullptr) {
				memset(_median, 0, sizeof(median_filter_t) * _loop);
			}
			break;
		}

		case FILTER_LINREG: {
			_regres = new regres_filter_t[_loop];
			if (_regres != nullptr) {
				memset(_regres, 0, sizeof(regres_filter_t) * _loop);
			}
			break;
		}
	}
}

/*******************************************************************************
* Function Name  : Destructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
Filter::~Filter()
{
	if (_median != nullptr) {
		delete [] _median;
	}

	if (_regres != nullptr) {
		delete [] _regres;
	}
}

/*******************************************************************************
* Function Name  : InvSqrt
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
float Filter::InvSqrt(float x)
{
	float xhalf = 0.5f*x;
	int i = *(int*)&x;
	i = 0x5f3759df - (i >> 1); 	// This line hides a LOT of math!
	x = *(float*)&i;
	x = x*(1.5f - xhalf*x*x); 	// repeat this statement for a better approximation

	return x;
}//InvSqrt

/*******************************************************************************
* Function Name  : Comparator
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int Filter::Comparator(const void *v1, const void * v2)
{
	if ((*((int*)v1)) < (*((int*)v2))) {
		return -1;
	}

	if ((*((int*)v1)) > (*((int*)v2))) {
		return 1;
	}

	return 0;
}//Comparator

/*******************************************************************************
* Function Name  : MedianInsert
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
float Filter::MedianInsert(int loop, float data)
{
	static float sorted[MEDIAN_DATALEN]={0};
	int i;

	// push the new data
	_median[loop].buffer[_median[loop].index++ % MEDIAN_DATALEN] = data;

	// copy the roll buffer
	for (i=0; i<MEDIAN_DATALEN; i++) {
		sorted[i] = _median[loop].buffer[i];
	}

	// sort the buffer
	qsort(sorted, MEDIAN_DATALEN, sizeof(float), Comparator);

	//return the median value
	return sorted[MEDIAN_INDEX];
}//MedianInsert

/*******************************************************************************
* Function Name  : LinRegress
* Description    : Linear Regression
*					y(x) = a + b*x, for n samples
*					The following assumes the standard deviations are unknown
*					for x and y
* Input          : None
* Output         : None
* Return         : Return a, b and r the regression coefficient
*******************************************************************************/
int Filter::LinRegress(regres_data_t *xy, int n, float *a, float *b, float *r)
{
#if 0
	float sxx, syy, sxy;
	float sumx=0, sumy=0;
	float sumx2=0, sumy2=0;
	float sumxy=0;
	int i;

	*a = 0;
	*b = 0;
	*r = 0;
	if (n < 2) {
		return 0;
	}

	// Compute some things we need
	for (i=0; i<n; i++) {
		sumx  += xy[i].x;
		sumy  += xy[i].y;
		sumx2 += (xy[i].x * xy[i].x);
		sumy2 += (xy[i].y * xy[i].y);
		sumxy += (xy[i].x * xy[i].y);
	}

	sxx = sumx2 - ((sumx * sumx) / n);
	syy = sumy2 - ((sumy * sumy) / n);
	sxy = sumxy - ((sumx * sumy) / n);

	// Infinite slope (b), non existant intercept (a)
	if (sxx < FLOAT_ZERO) {
		return 0;
	}

	// Compute the slope (b) and intercept (a)
	*b = sxy / sxx;
	*a = (sumy / n) - ((*b) * (sumx / n));

	// Compute the regression coefficient
	if (syy < FLOAT_ZERO) {
		*r = 1;
	} else {
		*r = sxy / sqrtf(sxx * syy);
	}

   return 1;
#else
	float Sxx=0, Syy=0, Sxy=0;
	float Mx=0, My=0;
	float xx, yy;
	int i;

	// Clear the values
	*a = 0;
	*b = 0;
	*r = 0;
	if (n < 2) {
		return 0;
	}

	// Compute mean values of x, y
	for (i=0; i<n; i++) {
		Mx += xy[i].x;
		My += xy[i].y;
	}
	Mx /= n;
	My /= n;

	// Compute Sxx, Syy and Sxy values
	for (i=0; i<n; i++) {
		xx = (xy[i].x - Mx);
		yy = (xy[i].y - My);

		Sxx += (xx * xx);
		Syy += (yy * yy);
		Sxy += (xx * yy);
	}

	// Infinite slope (b), non existant intercept (a)
	if (Sxx < FLOAT_ZERO) {
		return 0;
	}

	// Compute the slope (b) and intercept (a)
	*b = Sxy / Sxx;
	*a = My - ((*b) * Mx);

	// Compute the regression coefficient
	if ((Sxy < FLOAT_ZERO) || (Syy < FLOAT_ZERO)) {
		*r = 1;
	} else {
		*r = Sxy * InvSqrt(Sxx * Syy);
	}

   return 1;
#endif
}//LinRegress

/*******************************************************************************
* Function Name  : LinRegInsert
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Filter::LinRegInsert(int loop, float x, float y)
{
	_regres[loop].buffer[_regres[loop].index % REGRES_DATALEN].x = x;
	_regres[loop].buffer[_regres[loop].index % REGRES_DATALEN].y = y;
	_regres[loop].index++;
}//LinRegInsert

/*******************************************************************************
* Function Name  : insert
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
float Filter::insert(int loop, float data)
{
	switch (_type)
	{
		// median
		case FILTER_MEDIAN:
			return MedianInsert(loop, data);

		// regression
		case FILTER_LINREG:
		{
			float a, b, r;

			// insert the data
			LinRegInsert(loop, _regres[loop].index++, data);

			// try to fit
			if (LinRegress(_regres[loop].buffer, REGRES_DATALEN, &a, &b, &r)) {

				// check the fitting criteria
				if (r > 0.7f) {
					// return with estimated value
					return ((b * (_regres[loop].index + 1)) + a);
				}
			}
			break;
		}
	}

	return data;
}//insert
