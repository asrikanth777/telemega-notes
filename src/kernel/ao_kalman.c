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

more in the type and bitshifting
int32 -> 32 bits (4 bytes, 8 bits per byte)
int64 -> 64 bits (8 bytes, 8 bits per byte) difference by 
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
			ao_k_speed / (65536.0 * 16.0) //, 
			ao_accel / 16.0, 
			AO_K_STEP_100 / 65536.0,
			(ao_k_speed + (ao_k_t) ao_accel * AO_K_STEP_100) / (65536.0 * 16.0));
			/*
			the problem divides everything by 2^20, shifting each value by 20 bits, 
			(the shifting is more of a calculation speed thing, may be different in python.)
			*/
	}
#endif
// this part runs regardless of if AO_FLIGHT_TEST is there
	ao_k_height += ((ao_k_t) ao_speed * AO_K_STEP_100 + (ao_k_t) ao_accel * AO_K_STEP_2_2_100) >> 4;
	ao_k_speed += (ao_k_t) ao_accel * AO_K_STEP_100;
	/* - Aditya Srikanth
	same equations from earlier, but using ao_k_step_100. going by my previous assumption, if it has to do with steps and tickdiff multiplying to 50
	then maybe this is for tick differences less than 0.5 (100*0.5 = 50) or values that can't be handled in AO_FLIGHT_TEST
	or it is to add precision, which is why its for a much smaller tick difference using 100
	since this code runs regardless of AO_FLIGHT_TEST
	what i mean:	if AO_FLIGHT_TEST is there, it handles the biggest tick diff it can first, then does the STEP_100 part for extra precision
					if AO_FlIGHT_TEST isn't there, it starts with the smallest steps it can take for precision
					im assuming that maybe this takes longer? this is for shishir to answer maybe.
	*/
	
}

/* - Aditya Srikanth
Below is a function for calculating error height from kalman filter
*/

#if HAS_BARO
static void
ao_kalman_err_height(void) 	//check HAS_BARO (barometric data im assuming), if so, it runs the ao_kalman_err_height(void). 
{							// guessing this is going to calculate margin of error from real results vs kalman filter, so it needs actual data (HAS_BARO)

#if AO_ERROR_H_SQ_AVG 	//AO_ERROR_H_SQ_AVG prob stands for error height square average
	ao_v_t	e; 			// uninitialized variable {e} with type ao_v_t (int32_t)
#endif
	ao_v_t height_distrust; // regardless of AO_ERROR_H_SQ_AVG this is declared. height_distrust prob means how inaccurate height calculations are, will confirm later.
#if HAS_ACCEL
	ao_v_t	speed_distrust; // if acceleration data is given, it declares variable {speed_distrust}. hella times HAS_ACCEL is defined in this repo tho not sure why.
#endif

	ao_error_h = ao_sample_height - (ao_v_t) (ao_k_height >> 16); 	// goes regardless calculates actual difference between data and kalman
																	// followed by another bitewise shift????? i see it everywhere and dont understand what it does
																	// something to do with efficiency vs dividing normally maybe.

#if AO_ERROR_H_SQ_AVG // 	if AO_ERROR_H_SQ_AVG, e = ao_error_h (this is probably for sake of not having to rewrite ao_error_h
						//	over and ove again)
	e = ao_error_h;
	if (e < 0)			// we want error to always be positive (sign is not really important, it helps with calculation (overflow or smth))
		e = -e;
	if (e > 127)		// if e is too high, we just restrict it to one number for calculation sake
		e = 127;
	ao_error_h_sq_avg -= ao_error_h_sq_avg >> 4; // decays old h_sq_average
	ao_error_h_sq_avg += (e * e) >> 4; // adds the new error_hq_sq (e**2)
#endif
	/* - Aditya Srikanth
	the flight states are numbered in order, so this says if ao_flight_state is after ao_flight_drogue (parachuting),
	we ignore this part
	*/
	if (ao_flight_state >= ao_flight_drogue) 
		return;
	height_distrust = ao_sample_alt - AO_MAX_BARO_HEIGHT; // error tolerance = sample data - estimated max height
#if HAS_ACCEL
	/* speed is stored * 16, but we need to ramp between 248 and 328, so
	 * we want to multiply by 2. The result is a shift by 3.
	 */
	speed_distrust = (ao_speed - AO_MS_TO_SPEED(AO_MAX_BARO_SPEED)) >> (4 - 1); // error tolerance = sample data - estimated max speed
	if (speed_distrust > AO_MAX_SPEED_DISTRUST) // if calulated error tolerance is higher than estimated
		speed_distrust = AO_MAX_SPEED_DISTRUST; // then it becomes the same as estimated (for faster calculations?)
	if (speed_distrust > height_distrust)
		height_distrust = speed_distrust; // also, if speed distrust is heigher than height distrust, it changes the lower to match
#endif
	if (height_distrust > 0) { 
#ifdef AO_FLIGHT_TEST
		int	old_ao_error_h = ao_error_h; // overrides old error h
#endif
		if (height_distrust > 0x100) //clamping to cap decimal for faster calculations
			height_distrust = 0x100;
		ao_error_h = (ao_v_t) (((ao_k_t) ao_error_h * (0x100 - height_distrust)) >> 8); // more shifting calculations 
		/* - Aditya Srikanth
		basically, if height distrust low, ao_error_h stays close to original value, if high, ao_error_h scaled down
		*/
#ifdef AO_FLIGHT_TEST
		if (ao_flight_debug) {
			printf("over height %g over speed %g distrust: %g height: error %d -> %d\n", 
			       (double) (ao_sample_alt - AO_MAX_BARO_HEIGHT), // height distrust
			       (ao_speed - AO_MS_TO_SPEED(AO_MAX_BARO_SPEED)) / 16.0, // diff between current and max allowable speed
			       height_distrust / 256.0, // shifting like before if ao_flight_test is present
			       old_ao_error_h, // these are representing 
				   ao_error_h); // the change in error values 
		}
#endif
	}
}
#endif

#if HAS_BARO //if given barometric data, we do this stuff below
static void
ao_kalman_correct_baro(void)
/* - Aditya Srikanth
this function helps use error data to adjust height, speed, accel.
- first part runs the ao_kalman_err_height() function to find height error
- second part is similar to kalman predict function, which goes of tick difference.
	+ based on that difference, height is added to by the AO_BARO step multiplied by error (AO_BARO not defined anywhere)
	+ my assumption is that the 0, 1, 2 stand for derivations, 0 = height, 1 = speed, etc.

*/
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
#endif  // if neither tickdiff is met, smallest corrective measures are taken to calculate
	ao_k_height += (ao_k_t) AO_BARO_K0_100 * ao_error_h;
	ao_k_speed  += (ao_k_t) AO_BARO_K1_100 * ao_error_h;
	ao_k_accel  += (ao_k_t) AO_BARO_K2_100 * ao_error_h;
}
#endif

#if HAS_ACCEL // if acceleration data is given, then we start calculations on finding acceleration error

static void
ao_kalman_err_accel(void)
{
	ao_k_t	accel; //defines acceleration

	accel = (ao_config.accel_plus_g - ao_sample_accel) * ao_accel_scale ; // sensor to m/s2 conversion,

	/* - Aditya Srikanth
	total expected acceleration with gravity included - sample acceleration, this gives our "real" acceleration
	then this is multiplied by ao_acc
	*/

	/* Can't use ao_accel here as it is the pre-prediction value still (added by owner) */
	ao_error_a = (ao_v_t) ((accel - ao_k_accel) >> 16);
	// error acceleration calculated by newfound accel and predicted accel difference
}

#if !defined(FORCE_ACCEL) && HAS_BARO // checks for not having force accel data but has barometric data
static void
ao_kalman_correct_both(void) //correcting height and accel using the functions below
{
	ao_kalman_err_height();
	ao_kalman_err_accel();

#ifdef AO_FLIGHT_TEST
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 50) {
		if (ao_flight_debug) {
// Correct Speed ao_k_speed + (height error * scaling factor) + (acceleration error * scaling factor) = corrected speed calculation
			printf ("correct speed %g + (%g * %g) + (%g * %g) = %g\n",
				ao_k_speed / (65536.0 * 16.0),
				(double) ao_error_h, 
				AO_BOTH_K10_1 / 65536.0,
				(double) ao_error_a, 
				AO_BOTH_K11_1 / 65536.0,

				(ao_k_speed + (ao_k_t) AO_BOTH_K10_1 * ao_error_h + (ao_k_t) AO_BOTH_K11_1 * ao_error_a) / (65536.0 * 16.0));
		}
		ao_k_height += //kalman height = ao_both (??? maybe another timestep, no definition) * height error + ao_both_pt2deriv * accel error
			(ao_k_t) AO_BOTH_K00_1 * ao_error_h +
			(ao_k_t) AO_BOTH_K01_1 * ao_error_a;
		ao_k_speed +=//kalman speed = ao_both_pt1deriv * height error + ao_both_pt1_pt2derive * accel error
			(ao_k_t) AO_BOTH_K10_1 * ao_error_h +
			(ao_k_t) AO_BOTH_K11_1 * ao_error_a;
		ao_k_accel +=//kalman accel = ao_both_pt1derivSecond * height error + ao_both_pt1derivSecond_pt2deriv * accel error
			(ao_k_t) AO_BOTH_K20_1 * ao_error_h +
			(ao_k_t) AO_BOTH_K21_1 * ao_error_a;
		return;
	}
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 5) { 
		// literally same as before but with a smaller tick diff, goes from _1 to _10 (50*1 = 5*10)
		
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
	if (ao_flight_debug) { //like the previous 2, this does the same calculations, but if tick diff is less than 5
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

	/* - Aditya Srikanth
	some finishing notes, i am not sure what AO_BOTH stands for, since there is no other declaration or definition of these terms
	my assumption is that it stands for both barometer and accelerometer data but i can't say with certainty
	*/
}

#else

static void
ao_kalman_correct_accel(void) 
{
	ao_kalman_err_accel(); // runs function for calculating acceleration error

#ifdef AO_FLIGHT_TEST
	if ((AO_TICK_SIGNED) (ao_sample_tick - ao_sample_prev_tick) > 5) {
		ao_k_height +=(ao_k_t) AO_ACCEL_K0_10 * ao_error_a;
		ao_k_speed  += (ao_k_t) AO_ACCEL_K1_10 * ao_error_a;
		ao_k_accel  += (ao_k_t) AO_ACCEL_K2_10 * ao_error_a;
		return;
		/*
		nothing new here really, just calcualtions with low tick difference this time
		the numbers after k in the ao_k_t variables stand for derivations corresponding to
		height, speed (first deriv), and accel (second deriv).

		we know AO_ACCEL_KX to be kalman gains for acceleration (think of as scaling facotr), then multiplied by error to 
		calculate correction needed to existing factor

		*/
	}
#endif
// if no specified tick diff, it takes shortest timestep possbile, same as above
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

	// resets calculation by subtracing the additions to calculations, then
	// rewriting them to their change and and starting from there
}
#endif

void
ao_kalman(void)
{
// this function is like the main function of the program, and passes arguments for going through
// particular functions, strucutred through if else statements, to ensure a proper calculation using
// what is given
	ao_kalman_predict();
#if HAS_BARO
#if HAS_ACCEL
		// if both barometric and accel data are given/non-empty
	if (ao_flight_state <= ao_flight_coast) {
		// state before coasting stage of flight
#ifdef FORCE_ACCEL
		// if we have force accel, we only correct acceleration with the below function
		ao_kalman_correct_accel();
#else
		// otherwise we proceed as normal, using the both function that takes both barometric and accel data
		ao_kalman_correct_both();
#endif
	} else
#endif
		// if we are in or past coasting stage, we only care about correcting barometric data
		ao_kalman_correct_baro();
#else
#if HAS_ACCEL
	// if we doint have any barometric meassurements, we only correct acceleration with the correct_accel function
	ao_kalman_correct_accel();
#endif
#endif
	// converts fixed point estimates to floats to be used for further calcuations
	ao_height = (ao_v_t) from_fix(ao_k_height);
	ao_speed = (ao_v_t) from_fix(ao_k_speed);
	ao_accel = (ao_v_t) from_fix(ao_k_accel);
	if (ao_height > ao_max_height)
		ao_max_height = ao_height;
		// updates max altitude of flight (for the event of many test runs)
#if HAS_BARO
	ao_avg_height_scaled = ao_avg_height_scaled - ao_avg_height + ao_sample_height;
	// if we are given barometric data, we remove old average height data and replace it with
	// new sample data to get better height average calculation (updating it really)
#else
	ao_avg_height_scaled = ao_avg_height_scaled - ao_avg_height + ao_height;
	// otherwise we use kalman filter height data instead
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

/*
this part is a little confusing but from what gpt said, its for calculating more smoothly???
this is downscaling height_scaled to calculate avg_height and using byte shift for better calculations
this is kind of the equation i derived from it
ao_avg_height = ao_avg_height_scaled + (2^(x - 1) - 1) >> x { or / 2^x}
the lower the tick diff, the higher the scaling
*/


