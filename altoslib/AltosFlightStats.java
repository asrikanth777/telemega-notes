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

package org.altusmetrum.altoslib_11;

import java.io.*;

public class AltosFlightStats {
	public double		max_height;
	public double		max_gps_height;
	public double		max_speed;
	public double		max_acceleration;
	public double[]	state_speed = new double[AltosLib.ao_flight_invalid + 1];
	public double[]	state_accel = new double[AltosLib.ao_flight_invalid + 1];
	public int[]		state_count = new int[AltosLib.ao_flight_invalid + 1];
	public double[]	state_start = new double[AltosLib.ao_flight_invalid + 1];
	public double[]	state_end = new double[AltosLib.ao_flight_invalid + 1];
	public int		serial;
	public int		flight;
	public int		year, month, day;
	public int		hour, minute, second;
	public double		lat, lon;
	public double		pad_lat, pad_lon;
	public boolean		has_flight_data;
	public boolean		has_gps;
	public boolean		has_gps_sats;
	public boolean		has_gps_detail;
	public boolean		has_flight_adc;
	public boolean		has_battery;
	public boolean		has_rssi;
	public boolean		has_imu;
	public boolean		has_mag;
	public boolean		has_orient;
	public int		num_igniter;

	double landed_time(AltosFlightSeries series) {
		double	landed_state_time = AltosLib.MISSING;

		for (AltosTimeValue state : series.state_series) {
			if (state.value == AltosLib.ao_flight_landed) {
				landed_state_time = state.time;
				break;
			}
		}

		if (landed_state_time == AltosLib.MISSING)
			return AltosLib.MISSING;

		double landed_height = AltosLib.MISSING;
		for (AltosTimeValue height : series.height_series) {
			if (height.time >= landed_state_time) {
				landed_height = height.value;
				break;
			}
		}

		if (landed_height == AltosLib.MISSING)
			return AltosLib.MISSING;

		boolean	above = true;

		double	landed_time = AltosLib.MISSING;

		for (AltosTimeValue height : series.height_series) {
			if (height.value > landed_height + 10) {
				above = true;
			} else {
				if (above && Math.abs(height.value - landed_height) < 2) {
					above = false;
					landed_time = height.time;
				}
			}
		}
		return landed_time;
	}

	double boost_time(AltosFlightSeries series) {
		double 		boost_time = AltosLib.MISSING;
		double		boost_state_time = AltosLib.MISSING;

		for (AltosTimeValue state : series.state_series) {
			if (state.value >= AltosLib.ao_flight_boost && state.value <= AltosLib.ao_flight_landed) {
				boost_state_time = state.time;
				break;
			}
		}
		for (AltosTimeValue accel : series.accel_series) {
			if (accel.value < 1)
				boost_time = accel.time;
			if (boost_state_time != AltosLib.MISSING && accel.time >= boost_state_time)
				break;
		}
		return boost_time;
	}


	public AltosFlightStats(AltosFlightSeries series) throws InterruptedException, IOException {
		AltosCalData	cal_data = series.cal_data;
		double		boost_time = boost_time(series);
		double		end_time = 0;
		double		landed_time = landed_time(series);

		year = month = day = AltosLib.MISSING;
		hour = minute = second = AltosLib.MISSING;
		serial = flight = AltosLib.MISSING;
		lat = lon = AltosLib.MISSING;
		has_flight_data = false;
		has_gps = false;
		has_gps_sats = false;
		has_flight_adc = false;
		has_battery = false;
		has_rssi = false;
		has_imu = false;
		has_mag = false;
		has_orient = false;

		for (int s = AltosLib.ao_flight_startup; s <= AltosLib.ao_flight_landed; s++) {
			state_count[s] = 0;

			if (s == AltosLib.ao_flight_boost)
				state_start[s] = boost_time;
			else
				state_start[s] = series.state_series.time_of(s);
			if (s == AltosLib.ao_flight_landed)
				state_end[s] = landed_time;
			else
				state_end[s] = series.state_series.time_of(s+1);

			if (series.speed_series != null)
				state_speed[s] = series.speed_series.average(state_start[s], state_end[s]);

			if (series.accel_series != null)
				state_accel[s] = series.accel_series.average(state_start[s], state_end[s]);
		}

		serial = cal_data.serial;
		flight = cal_data.flight;

		has_battery = series.battery_voltage_series != null;
		has_flight_adc = series.main_voltage_series != null;
		has_rssi = series.rssi_series != null;
		has_flight_data = series.pressure_series != null;

		AltosGPS gps = series.cal_data.gps_pad;

		if (gps != null) {
			year = gps.year;
			month = gps.month;
			day = gps.day;
			hour = gps.hour;
			minute = gps.minute;
			second = gps.second;
			has_gps = true;
			lat = pad_lat = gps.lat;
			lon = pad_lon = gps.lon;
			for (AltosGPSTimeValue gtv : series.gps_series) {
				gps = gtv.gps;
				if (gps.locked && gps.nsat >= 4) {
					lat = gps.lat;
					lon = gps.lon;
				}
			}

		}

		max_height = AltosLib.MISSING;
		if (series.height_series != null)
			max_height = series.height_series.max();
		max_speed = AltosLib.MISSING;
		if (series.speed_series != null) {
			max_speed = series.speed_series.max(state_start[AltosLib.ao_flight_boost], state_start[AltosLib.ao_flight_drogue]);
			if (max_speed == AltosLib.MISSING)
				max_speed = series.speed_series.max();
		}
		max_acceleration = AltosLib.MISSING;
		if (series.accel_series != null) {
			max_acceleration = series.accel_series.max(state_start[AltosLib.ao_flight_boost], state_start[AltosLib.ao_flight_drogue]);
			if (max_acceleration == AltosLib.MISSING)
				max_acceleration = series.accel_series.max();
		}
		max_gps_height = AltosLib.MISSING;
		if (series.gps_height != null)
			max_gps_height = series.gps_height.max();

	}
}
