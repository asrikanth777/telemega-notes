/*
 * Copyright © 2011 Keith Packard <keithp@keithp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

/* - Aditya Srikanth
imports for ao.h, ao_flight.h, ao_sample.h
no clue where ao_kalman.h is currently other than in .gitignore
*/
#ifndef AO_FLIGHT_TEST
#include "ao.h"
#include "ao_flight.h"
#endif

#include "ao_sample.h"
#include "ao_kalman.h"

/* - Aditya Srikanth
static variable declarations, if we are doing in python, we can ignore this
ao_k_t is a type of int64_t, giving int range of 2^-63 to 2^63 - 1
then we have uninitialized variables for height, speed, accel
*/

static ao_k_t		ao_k_height;
static ao_k_t		ao_k_speed;
static ao_k_t		ao_k_accel;

/* - Aditya Srikanth
these are increments 
the first type is defined by 1 divided by the number given, or also AO_K_STEP_(x) = 1/x
the second type is taking the result from before, squaring it, and then dividing it by 2
- for example: 0.00005 = (0.01**2) / 2

the steps have something to do with integration/derivation i think
for example, integral(x) = x**2 / 2

this is to help with distance and velocity calculations, and based on the tick range, it uses the step accordingly, 
since it would be difficult to calculate for each incrementation
think of it like a staircase going from height a to height b, this is to help set the number of "steps" needed
*/

#define AO_K_STEP_100		to_fix_v(0.01)
#define AO_K_STEP_2_2_100	to_fix_v(0.00005)

#define AO_K_STEP_10		to_fix_v(0.1)
#define AO_K_STEP_2_2_10	to_fix_v(0.005)

#define AO_K_STEP_1			to_fix_v(1)
#define AO_K_STEP_2_2_1		to_fix_v(0.5)

/* - Aditya Srikanth
more variable declarations, but these aren't static. will look into where else they may be used in the repo
ao_v_t is an int32_t type and is used to define ao_height, ao_speed, ao_accel, ao_max_height, etc.
however, ao_avg_height_scaled is in ao_k_t from earlier

my assumption is that the ao_k is for kalman filter and plain ao_ is from sample data? not sure
*/

ao_v_t				ao_height;
ao_v_t				ao_speed; //int32 ---> 2**-31 to 2**31 - 1
ao_v_t				ao_accel;
ao_v_t				ao_max_height;
static ao_k_t		ao_avg_height_scaled;
ao_v_t				ao_avg_height;
ao_v_t				ao_error_h;

/* - Aditya Srikanth
some conditional statements down here
1) first one says if the program doesnt have HAS_ACCEL or AO_FLIGHT_TEST, then it defines a macro named AO_ERROR_H_SQ_AVG with a value of 1
2) second one says that if AO_ERROR_H_SQ_AVG is non-zero (or 1 as we defined earlier), an uninitialized variable of
   type ao_v_t (int32_t) is created and named ao_error_h_sq_avg
3) third one says that if the program has HAS_ACCEL, it defines a variable named ao_error_a

Assumptions of mine:
I want to say that these are incase that the data given doesn't have everything needed and used to make work arounds, especially for 1 and 2.
for 3, it is just setting the stage for calculating error of acceleration if given.
*/

#if !HAS_ACCEL || AO_FLIGHT_TEST
#define AO_ERROR_H_SQ_AVG	1
#endif

#if AO_ERROR_H_SQ_AVG
ao_v_t			ao_error_h_sq_avg;
#endif

#if HAS_ACCEL
ao_v_t			ao_error_a;
#endif

/* - Aditya Srikanth
ao_kalman_predict is what i presume to be the filter's prediction on the rockets position, vel, acc, and more. 
*/

static void
ao_kalman_predict(void)
{
#ifdef AO_FLIGHT_TEST // checking if AO_FLIGHT_TEST is there (some reason its in the .gitignore dont know why)
	// - Aditya Srikanth
	// here we can assume tick to be steps, so this is counting if the difference between the current tick and previous tick is greater than 50 (seconds? milliseconds?)
	// units are unclear, it could be nanoseconds for all i know
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 50) {
		// this adds to our original height using a speed and acceleration
		// think of distance equation: D = D0 + v*t + 1/2at^2
		ao_k_height += ((ao_k_t) ao_speed * AO_K_STEP_1 + (ao_k_t) ao_accel * AO_K_STEP_2_2_1) >> 4; //bitwise shift, not exactly sure what it is for, something to do with binary representaiton
		// speed equation: Vf = V0 + at
		ao_k_speed += (ao_k_t) ao_accel * AO_K_STEP_1;

		return;
	}
	// - Aditya Srikanth
	// basically the same thing here as above
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 5) {
		ao_k_height += ((ao_k_t) ao_speed * AO_K_STEP_10 +
				(ao_k_t) ao_accel * AO_K_STEP_2_2_10) >> 4;
		ao_k_speed += (ao_k_t) ao_accel * AO_K_STEP_10;

		return;
	}

	/* - Aditya Srikanth
	it seems that 50 is a significant value here, wondering why????
	first if statement: tick diff > 50 ------> use AO_K_STEP_1
	second if statement: tick diff > 5 ------> use AO_K_STEP_10 (5*10=50)
	what is important about this 
	*/

	
	if (ao_flight_debug) {
		printf ("predict speed %g + (%g * %g) = %g\n",
			// (aokspeed / 2^20) + (aoaccel*timestep/2^20) = speed prediction
			ao_k_speed / (65536.0 * 16.0), 
			ao_accel / 16.0, 
			AO_K_STEP_100 / 65536.0,
			(ao_k_speed + (ao_k_t) ao_accel * AO_K_STEP_100) / (65536.0 * 16.0));
			/*
			its taking the speed equation and dividing by 2^20???? asked gpt and it said something about
			handling fixed point arithmetic by scaling?
			(this is more a of a cs thing i think, im not sure what it does or how it helps with calculation.)
			*/
	}
#endif
// this part runs regardless of if AO_FLIGHT_TEST is there
	ao_k_height += ((ao_k_t) ao_speed * AO_K_STEP_100 + (ao_k_t) ao_accel * AO_K_STEP_2_2_100) >> 4;
	ao_k_speed += (ao_k_t) ao_accel * AO_K_STEP_100;
	// same equations from earlier, but using ao_k_step_100. going by my previous assumption, if it has to do with steps and tickdiff multiplying to 50
	// then maybe this is for tick differences less than 0.5 (100*0.5 = 50) or values that can't be handled in AO_FLIGHT_TEST
	// or it is to add precision, which is why its for a much smaller tick difference using 100
	// since this code runs regardless of AO_FLIGHT_TEST
	
}

/* - Aditya Srikanth

*/

#if HAS_BARO
static void
ao_kalman_err_height(void)
{
#if AO_ERROR_H_SQ_AVG
	ao_v_t	e;
#endif
	ao_v_t height_distrust;
#if HAS_ACCEL
	ao_v_t	speed_distrust;
#endif

	ao_error_h = ao_sample_height - (ao_v_t) (ao_k_height >> 16);

#if AO_ERROR_H_SQ_AVG
	e = ao_error_h;
	if (e < 0)
		e = -e;
	if (e > 127)
		e = 127;
	ao_error_h_sq_avg -= ao_error_h_sq_avg >> 4;
	ao_error_h_sq_avg += (e * e) >> 4;
#endif

	if (ao_flight_state >= ao_flight_drogue)
		return;
	height_distrust = ao_sample_alt - AO_MAX_BARO_HEIGHT;
#if HAS_ACCEL
	/* speed is stored * 16, but we need to ramp between 248 and 328, so
	 * we want to multiply by 2. The result is a shift by 3.
	 */
	speed_distrust = (ao_speed - AO_MS_TO_SPEED(AO_MAX_BARO_SPEED)) >> (4 - 1);
	if (speed_distrust > AO_MAX_SPEED_DISTRUST)
		speed_distrust = AO_MAX_SPEED_DISTRUST;
	if (speed_distrust > height_distrust)
		height_distrust = speed_distrust;
#endif
	if (height_distrust > 0) {
#ifdef AO_FLIGHT_TEST
		int	old_ao_error_h = ao_error_h;
#endif
		if (height_distrust > 0x100)
			height_distrust = 0x100;
		ao_error_h = (ao_v_t) (((ao_k_t) ao_error_h * (0x100 - height_distrust)) >> 8);
#ifdef AO_FLIGHT_TEST
		if (ao_flight_debug) {
			printf("over height %g over speed %g distrust: %g height: error %d -> %d\n",
			       (double) (ao_sample_alt - AO_MAX_BARO_HEIGHT),
			       (ao_speed - AO_MS_TO_SPEED(AO_MAX_BARO_SPEED)) / 16.0,
			       height_distrust / 256.0,
			       old_ao_error_h, ao_error_h);
		}
#endif
	}
}
#endif

#if HAS_BARO
static void
ao_kalman_correct_baro(void)
{
	ao_kalman_err_height();
#ifdef AO_FLIGHT_TEST
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 50) {
		ao_k_height += (ao_k_t) AO_BARO_K0_1 * ao_error_h;
		ao_k_speed  += (ao_k_t) AO_BARO_K1_1 * ao_error_h;
		ao_k_accel  += (ao_k_t) AO_BARO_K2_1 * ao_error_h;
		return;
	}
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 5) {
		ao_k_height += (ao_k_t) AO_BARO_K0_10 * ao_error_h;
		ao_k_speed  += (ao_k_t) AO_BARO_K1_10 * ao_error_h;
		ao_k_accel  += (ao_k_t) AO_BARO_K2_10 * ao_error_h;
		return;
	}
#endif
	ao_k_height += (ao_k_t) AO_BARO_K0_100 * ao_error_h;
	ao_k_speed  += (ao_k_t) AO_BARO_K1_100 * ao_error_h;
	ao_k_accel  += (ao_k_t) AO_BARO_K2_100 * ao_error_h;
}
#endif

#if HAS_ACCEL

static void
ao_kalman_err_accel(void)
{
	ao_k_t	accel;

	accel = (ao_config.accel_plus_g - ao_sample_accel) * ao_accel_scale;

	/* Can't use ao_accel here as it is the pre-prediction value still */
	ao_error_a = (ao_v_t) ((accel - ao_k_accel) >> 16);
}

#if !defined(FORCE_ACCEL) && HAS_BARO
static void
ao_kalman_correct_both(void)
{
	ao_kalman_err_height();
	ao_kalman_err_accel();

#ifdef AO_FLIGHT_TEST
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 50) {
		if (ao_flight_debug) {
			printf ("correct speed %g + (%g * %g) + (%g * %g) = %g\n",
				ao_k_speed / (65536.0 * 16.0),
				(double) ao_error_h, AO_BOTH_K10_1 / 65536.0,
				(double) ao_error_a, AO_BOTH_K11_1 / 65536.0,
				(ao_k_speed +
				 (ao_k_t) AO_BOTH_K10_1 * ao_error_h +
				 (ao_k_t) AO_BOTH_K11_1 * ao_error_a) / (65536.0 * 16.0));
		}
		ao_k_height +=
			(ao_k_t) AO_BOTH_K00_1 * ao_error_h +
			(ao_k_t) AO_BOTH_K01_1 * ao_error_a;
		ao_k_speed +=
			(ao_k_t) AO_BOTH_K10_1 * ao_error_h +
			(ao_k_t) AO_BOTH_K11_1 * ao_error_a;
		ao_k_accel +=
			(ao_k_t) AO_BOTH_K20_1 * ao_error_h +
			(ao_k_t) AO_BOTH_K21_1 * ao_error_a;
		return;
	}
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 5) {
		if (ao_flight_debug) {
			printf ("correct speed %g + (%g * %g) + (%g * %g) = %g\n",
				ao_k_speed / (65536.0 * 16.0),
				(double) ao_error_h, AO_BOTH_K10_10 / 65536.0,
				(double) ao_error_a, AO_BOTH_K11_10 / 65536.0,
				(ao_k_speed +
				 (ao_k_t) AO_BOTH_K10_10 * ao_error_h +
				 (ao_k_t) AO_BOTH_K11_10 * ao_error_a) / (65536.0 * 16.0));
		}
		ao_k_height +=
			(ao_k_t) AO_BOTH_K00_10 * ao_error_h +
			(ao_k_t) AO_BOTH_K01_10 * ao_error_a;
		ao_k_speed +=
			(ao_k_t) AO_BOTH_K10_10 * ao_error_h +
			(ao_k_t) AO_BOTH_K11_10 * ao_error_a;
		ao_k_accel +=
			(ao_k_t) AO_BOTH_K20_10 * ao_error_h +
			(ao_k_t) AO_BOTH_K21_10 * ao_error_a;
		return;
	}
	if (ao_flight_debug) {
		printf ("correct speed %g + (%g * %g) + (%g * %g) = %g\n",
			ao_k_speed / (65536.0 * 16.0),
			(double) ao_error_h, AO_BOTH_K10_100 / 65536.0,
			(double) ao_error_a, AO_BOTH_K11_100 / 65536.0,
			(ao_k_speed +
			 (ao_k_t) AO_BOTH_K10_100 * ao_error_h +
			 (ao_k_t) AO_BOTH_K11_100 * ao_error_a) / (65536.0 * 16.0));
	}
#endif
	ao_k_height +=
		(ao_k_t) AO_BOTH_K00_100 * ao_error_h +
		(ao_k_t) AO_BOTH_K01_100 * ao_error_a;
	ao_k_speed +=
		(ao_k_t) AO_BOTH_K10_100 * ao_error_h +
		(ao_k_t) AO_BOTH_K11_100 * ao_error_a;
	ao_k_accel +=
		(ao_k_t) AO_BOTH_K20_100 * ao_error_h +
		(ao_k_t) AO_BOTH_K21_100 * ao_error_a;
}

#else

static void
ao_kalman_correct_accel(void)
{
	ao_kalman_err_accel();

#ifdef AO_FLIGHT_TEST
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 5) {
		ao_k_height +=(ao_k_t) AO_ACCEL_K0_10 * ao_error_a;
		ao_k_speed  += (ao_k_t) AO_ACCEL_K1_10 * ao_error_a;
		ao_k_accel  += (ao_k_t) AO_ACCEL_K2_10 * ao_error_a;
		return;
	}
#endif
	ao_k_height += (ao_k_t) AO_ACCEL_K0_100 * ao_error_a;
	ao_k_speed  += (ao_k_t) AO_ACCEL_K1_100 * ao_error_a;
	ao_k_accel  += (ao_k_t) AO_ACCEL_K2_100 * ao_error_a;
}

#endif /* else FORCE_ACCEL */
#endif /* HAS_ACCEL */

#if !HAS_BARO
static ao_k_t	ao_k_height_prev;
static ao_k_t	ao_k_speed_prev;

/*
 * While in pad mode without a barometric sensor, remove accumulated
 * speed and height values to reduce the effect of systematic sensor
 * error
 */
void
ao_kalman_reset_accumulate(void)
{
	ao_k_height -= ao_k_height_prev;
	ao_k_speed -= ao_k_speed_prev;
	ao_k_height_prev = ao_k_height;
	ao_k_speed_prev = ao_k_speed;
}
#endif

void
ao_kalman(void)
{
	ao_kalman_predict();
#if HAS_BARO
#if HAS_ACCEL
	if (ao_flight_state <= ao_flight_coast) {
#ifdef FORCE_ACCEL
		ao_kalman_correct_accel();
#else
		ao_kalman_correct_both();
#endif
	} else
#endif
		ao_kalman_correct_baro();
#else
#if HAS_ACCEL
	ao_kalman_correct_accel();
#endif
#endif
	ao_height = (ao_v_t) from_fix(ao_k_height);
	ao_speed = (ao_v_t) from_fix(ao_k_speed);
	ao_accel = (ao_v_t) from_fix(ao_k_accel);
	if (ao_height > ao_max_height)
		ao_max_height = ao_height;
#if HAS_BARO
	ao_avg_height_scaled = ao_avg_height_scaled - ao_avg_height + ao_sample_height;
#else
	ao_avg_height_scaled = ao_avg_height_scaled - ao_avg_height + ao_height;
#endif
#ifdef AO_FLIGHT_TEST
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 50)
		ao_avg_height = (ao_avg_height_scaled + 1) >> 1;
	else if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 5)
		ao_avg_height = (ao_avg_height_scaled + 7) >> 4;
	else 
#endif
		ao_avg_height = (ao_v_t) ((ao_avg_height_scaled + 63) >> 7);
}
