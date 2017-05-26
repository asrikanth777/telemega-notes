/*
 * Copyright © 2013 Keith Packard <keithp@keithp.com>
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
import java.util.*;
import java.text.*;

class AltosTelemetryNullListener extends AltosDataListener {
	public void set_rssi(int rssi, int status) { }
	public void set_received_time(long received_time) { }

	public void set_acceleration(double accel) { }
	public void set_pressure(double pa) { }
	public void set_thrust(double N) { }

	public void set_kalman(double height, double speed, double accel) { }

	public void set_temperature(double deg_c) { }
	public void set_battery_voltage(double volts) { }

	public void set_apogee_voltage(double volts) { }
	public void set_main_voltage(double volts) { }

	public void set_gps(AltosGPS gps) { }

	public void set_orient(double orient) { }
	public void set_gyro(double roll, double pitch, double yaw) { }
	public void set_accel_ground(double along, double across, double through) { }
	public void set_accel(double along, double across, double through) { }
	public void set_mag(double along, double across, double through) { }
	public void set_pyro_voltage(double volts) { }
	public void set_ignitor_voltage(double[] voltage) { }
	public void set_pyro_fired(int pyro_mask) { }
	public void set_companion(AltosCompanion companion) { }

	public boolean cal_data_complete() {
		/* All telemetry packets */
		if (cal_data.serial == AltosLib.MISSING)
			return false;

		if (cal_data.boost_tick == AltosLib.MISSING)
			return false;

		/*
		 * TelemetryConfiguration:
		 *
		 * device_type, flight, config version, log max,
		 * flight params, callsign and version
		 */
		if (cal_data.device_type == AltosLib.MISSING)
			return false;

		/*
		 * TelemetrySensor or TelemetryMegaData:
		 *
		 * ground_accel, accel+/-, ground pressure
		 */
		if (cal_data.ground_pressure == AltosLib.MISSING)
			return false;

		/*
		 * TelemetryLocation
		 */
		if (AltosLib.has_gps(cal_data.device_type) && cal_data.gps_ground_altitude == AltosLib.MISSING)
			return false;

		return true;
	}

	public AltosTelemetryNullListener(AltosCalData cal_data) {
		super(cal_data);
	}
}

public class AltosTelemetryFile implements AltosRecordSet {

	AltosTelemetryIterable	telems;
	AltosCalData		cal_data;

	public void write_comments(PrintStream out) {
	}

	public void write(PrintStream out) {
	}

	/* Construct cal data by walking through the telemetry data until we've found everything available */
	public AltosCalData cal_data() {
		if (cal_data == null) {
			cal_data = new AltosCalData();
			AltosTelemetryNullListener l = new AltosTelemetryNullListener(cal_data);

			for (AltosTelemetry telem : telems) {
				telem.provide_data(l, cal_data);
				if (l.cal_data_complete())
					break;
			}
			System.out.printf("Telemetry boost tick %d\n", cal_data.boost_tick);
		}
		return cal_data;
	}

	public void capture_series(AltosDataListener listener) {
		AltosCalData	cal_data = cal_data();

		for (AltosTelemetry telem : telems) {
			int tick = telem.tick();
			cal_data.set_tick(tick);
			if (cal_data.time() >= -1)
				telem.provide_data(listener, cal_data);
		}
	}

	public AltosTelemetryFile(FileInputStream input) {
		telems = new AltosTelemetryIterable(input);
	}
}
