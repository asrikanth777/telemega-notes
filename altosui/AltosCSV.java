/*
 * Copyright © 2010 Keith Packard <keithp@keithp.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
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

package altosui;

import java.lang.*;
import java.io.*;
import java.text.*;
import java.util.*;

public class AltosCSV implements AltosWriter {
	File			name;
	PrintStream		out;
	boolean			header_written;
	boolean			seen_boost;
	int			boost_tick;
	LinkedList<AltosRecord>	pad_records;
	AltosState		state;

	static final int ALTOS_CSV_VERSION = 5;

	/* Version 4 format:
	 *
	 * General info
	 *	version number
	 *	serial number
	 *	flight number
	 *	callsign
	 *	time (seconds since boost)
	 *	clock (tick count / 100)
	 *	rssi
	 *	link quality
	 *
	 * Flight status
	 *	state
	 *	state name
	 *
	 * Basic sensors
	 *	acceleration (m/s²)
	 *	pressure (mBar)
	 *	altitude (m)
	 *	height (m)
	 *	accelerometer speed (m/s)
	 *	barometer speed (m/s)
	 *	temp (°C)
	 *	battery (V)
	 *	drogue (V)
	 *	main (V)
	 *
	 * Advanced sensors (if available)
	 *	accel_x (m/s²)
	 *	accel_y (m/s²)
	 *	accel_z (m/s²)
	 *	gyro_x (d/s)
	 *	gyro_y (d/s)
	 *	gyro_z (d/s)
	 *	mag_x (g)
	 *	mag_y (g)
	 *	mag_z (g)
	 *
	 * GPS data (if available)
	 *	connected (1/0)
	 *	locked (1/0)
	 *	nsat (used for solution)
	 *	latitude (°)
	 *	longitude (°)
	 *	altitude (m)
	 *	year (e.g. 2010)
	 *	month (1-12)
	 *	day (1-31)
	 *	hour (0-23)
	 *	minute (0-59)
	 *	second (0-59)
	 *	from_pad_dist (m)
	 *	from_pad_azimuth (deg true)
	 *	from_pad_range (m)
	 *	from_pad_elevation (deg from horizon)
	 *	hdop
	 *
	 * GPS Sat data
	 *	C/N0 data for all 32 valid SDIDs
	 *
	 * Companion data
	 *	companion_id (1-255. 10 is TeleScience)
	 *	time of last companion data (seconds since boost)
	 *	update_period (0.1-2.55 minimum telemetry interval)
	 *	channels (0-12)
	 *	channel data for all 12 possible channels
	 */

	void write_general_header() {
		out.printf("version,serial,flight,call,time,clock,rssi,lqi");
	}

	void write_general(AltosRecord record) {
		out.printf("%s, %d, %d, %s, %8.2f, %8.2f, %4d, %3d",
			   ALTOS_CSV_VERSION, record.serial, record.flight, record.callsign,
			   (double) record.time, (double) record.tick / 100.0,
			   record.rssi,
			   record.status & 0x7f);
	}

	void write_flight_header() {
		out.printf("state,state_name");
	}

	void write_flight(AltosRecord record) {
		out.printf("%d,%8s", record.state, record.state());
	}

	void write_basic_header() {
		out.printf("acceleration,pressure,altitude,height,accel_speed,baro_speed,temperature,battery_voltage,drogue_voltage,main_voltage");
	}

	void write_basic(AltosRecord record) {
		out.printf("%8.2f,%10.2f,%8.2f,%8.2f,%8.2f,%8.2f,%5.1f,%5.2f,%5.2f,%5.2f",
			   record.acceleration(),
			   record.raw_pressure(),
			   record.raw_altitude(),
			   record.raw_height(),
			   record.accel_speed(),
			   state.baro_speed,
			   record.temperature(),
			   record.battery_voltage(),
			   record.drogue_voltage(),
			   record.main_voltage());
	}

	void write_advanced_header() {
		out.printf("accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z");
	}

	void write_advanced(AltosRecord record) {
		AltosIMU	imu = record.imu;
		AltosMag	mag = record.mag;

		if (imu == null)
			imu = new AltosIMU();
		if (mag == null)
			mag = new AltosMag();
		out.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d",
			   imu.accel_x, imu.accel_y, imu.accel_z,
			   imu.gyro_x, imu.gyro_y, imu.gyro_z,
			   mag.x, mag.y, mag.z);
	}

	void write_gps_header() {
		out.printf("connected,locked,nsat,latitude,longitude,altitude,year,month,day,hour,minute,second,pad_dist,pad_range,pad_az,pad_el,hdop");
	}

	void write_gps(AltosRecord record) {
		AltosGPS	gps = record.gps;
		if (gps == null)
			gps = new AltosGPS();

		AltosGreatCircle from_pad = state.from_pad;
		if (from_pad == null)
			from_pad = new AltosGreatCircle();

		out.printf("%2d,%2d,%3d,%12.7f,%12.7f,%6d,%5d,%3d,%3d,%3d,%3d,%3d,%9.0f,%9.0f,%4.0f,%4.0f,%6.1f",
			   gps.connected?1:0,
			   gps.locked?1:0,
			   gps.nsat,
			   gps.lat,
			   gps.lon,
			   gps.alt,
			   gps.year,
			   gps.month,
			   gps.day,
			   gps.hour,
			   gps.minute,
			   gps.second,
			   from_pad.distance,
			   state.range,
			   from_pad.bearing,
			   state.elevation,
			   gps.hdop);
	}

	void write_gps_sat_header() {
		for(int i = 1; i <= 32; i++) {
			out.printf("sat%02d", i);
			if (i != 32)
				out.printf(",");
		}
	}

	void write_gps_sat(AltosRecord record) {
		AltosGPS	gps = record.gps;
		for(int i = 1; i <= 32; i++) {
			int	c_n0 = 0;
			if (gps != null && gps.cc_gps_sat != null) {
				for(int j = 0; j < gps.cc_gps_sat.length; j++)
					if (gps.cc_gps_sat[j].svid == i) {
						c_n0 = gps.cc_gps_sat[j].c_n0;
						break;
					}
			}
			out.printf ("%3d", c_n0);
			if (i != 32)
				out.printf(",");
		}
	}

	void write_companion_header() {
		out.printf("companion_id,companion_time,companion_update,companion_channels");
		for (int i = 0; i < 12; i++)
			out.printf(",companion_%02d", i);
	}

	void write_companion(AltosRecord record) {
		AltosRecordCompanion companion = record.companion;

		int	channels_written = 0;
		if (companion == null) {
			out.printf("0,0,0,0");
		} else {
			out.printf("%3d,%5.2f,%5.2f,%2d",
				   companion.board_id,
				   (companion.tick - boost_tick) / 100.0,
				   companion.update_period / 100.0,
				   companion.channels);
			for (; channels_written < companion.channels; channels_written++)
				out.printf(",%5d", companion.companion_data[channels_written]);
		}
		for (; channels_written < 12; channels_written++)
			out.printf(",0");
	}

	void write_header(boolean advanced, boolean gps, boolean companion) {
		out.printf("#"); write_general_header();
		out.printf(","); write_flight_header();
		out.printf(","); write_basic_header();
		if (advanced)
			out.printf(","); write_advanced_header();
		if (gps) {
			out.printf(","); write_gps_header();
			out.printf(","); write_gps_sat_header();
		}
		if (companion) {
			out.printf(","); write_companion_header();
		}
		out.printf ("\n");
	}

	void write_one(AltosRecord record) {
		state = new AltosState(record, state);
		write_general(record); out.printf(",");
		write_flight(record); out.printf(",");
		write_basic(record); out.printf(",");
		if (record.imu != null || record.mag != null)
			write_advanced(record);
		if (record.gps != null) {
			out.printf(",");
			write_gps(record); out.printf(",");
			write_gps_sat(record);
		}
		if (record.companion != null) {
			out.printf(",");
			write_companion(record);
		}
		out.printf ("\n");
	}

	void flush_pad() {
		while (!pad_records.isEmpty()) {
			write_one (pad_records.remove());
		}
	}

	public void write(AltosRecord record) {
		if (record.state == Altos.ao_flight_startup)
			return;
		if (!header_written) {
			write_header(record.imu != null || record.mag != null,
				     record.gps != null, record.companion != null);
			header_written = true;
		}
		if (!seen_boost) {
			if (record.state >= Altos.ao_flight_boost) {
				seen_boost = true;
				boost_tick = record.tick;
				flush_pad();
			}
		}
		if (seen_boost)
			write_one(record);
		else
			pad_records.add(record);
	}

	public PrintStream out() {
		return out;
	}

	public void close() {
		if (!pad_records.isEmpty()) {
			boost_tick = pad_records.element().tick;
			flush_pad();
		}
		out.close();
	}

	public void write(AltosRecordIterable iterable) {
		iterable.write_comments(out());
		for (AltosRecord r : iterable)
			write(r);
	}

	public AltosCSV(PrintStream in_out, File in_name) {
		name = in_name;
		out = in_out;
		pad_records = new LinkedList<AltosRecord>();
	}

	public AltosCSV(File in_name) throws FileNotFoundException {
		this(new PrintStream(in_name), in_name);
	}

	public AltosCSV(String in_string) throws FileNotFoundException {
		this(new File(in_string));
	}
}
