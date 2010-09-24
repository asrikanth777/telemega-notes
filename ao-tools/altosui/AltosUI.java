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

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import javax.swing.table.*;
import java.io.*;
import java.util.*;
import java.text.*;
import java.util.prefs.*;
import java.util.concurrent.LinkedBlockingQueue;

import altosui.Altos;
import altosui.AltosSerial;
import altosui.AltosSerialMonitor;
import altosui.AltosRecord;
import altosui.AltosTelemetry;
import altosui.AltosState;
import altosui.AltosDeviceDialog;
import altosui.AltosPreferences;
import altosui.AltosLog;
import altosui.AltosVoice;
import altosui.AltosFlightInfoTableModel;
import altosui.AltosChannelMenu;
import altosui.AltosFlashUI;
import altosui.AltosLogfileChooser;
import altosui.AltosCSVUI;
import altosui.AltosLine;
import altosui.AltosStatusTable;
import altosui.AltosInfoTable;

import libaltosJNI.*;

public class AltosUI extends JFrame {
	private int channel = -1;

	private AltosStatusTable flightStatus;
	private AltosInfoTable flightInfo;
	private AltosSerial serial_line;
	private AltosLog altos_log;
	private Box vbox;

	private Font statusFont = new Font("SansSerif", Font.BOLD, 24);
	private Font infoLabelFont = new Font("SansSerif", Font.PLAIN, 14);
	private Font infoValueFont = new Font("Monospaced", Font.PLAIN, 14);

	public AltosVoice voice = new AltosVoice();

	public static boolean load_library(Frame frame) {
		if (!AltosDevice.load_library()) {
			JOptionPane.showMessageDialog(frame,
						      String.format("No AltOS library in \"%s\"",
								    System.getProperty("java.library.path","<undefined>")),
						      "Cannot load device access library",
						      JOptionPane.ERROR_MESSAGE);
			return false;
		}
		return true;
	}

	public AltosUI() {

		load_library(null);

		String[] statusNames = { "Height (m)", "State", "RSSI (dBm)", "Speed (m/s)" };
		Object[][] statusData = { { "0", "pad", "-50", "0" } };

		java.net.URL imgURL = AltosUI.class.getResource("/altus-metrum-16x16.jpg");
		if (imgURL != null)
			setIconImage(new ImageIcon(imgURL).getImage());

		AltosPreferences.init(this);

		vbox = Box.createVerticalBox();
		this.add(vbox);

		flightStatus = new AltosStatusTable(this);

		vbox.add(flightStatus);

		flightInfo = new AltosInfoTable();
		vbox.add(flightInfo.box());

		setTitle("AltOS");

		createMenu();

		serial_line = new AltosSerial();
		altos_log = new AltosLog(serial_line);
		int dpi = Toolkit.getDefaultToolkit().getScreenResolution();
		this.setSize(new Dimension (flightInfo.width(),
					    flightStatus.height() + flightInfo.height()));
		this.validate();
		setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});
		voice.speak("Rocket flight monitor ready.");
	}

	void show(AltosState state, int crc_errors) {
		if (state != null) {
			flightStatus.set(state);
			flightInfo.show(state, crc_errors);
		}
	}

	class IdleThread extends Thread {

		boolean	started;
		private AltosState state;
		int	reported_landing;
		int	report_interval;
		long	report_time;

		public synchronized void report(boolean last) {
			if (state == null)
				return;

			/* reset the landing count once we hear about a new flight */
			if (state.state < Altos.ao_flight_drogue)
				reported_landing = 0;

			/* Shut up once the rocket is on the ground */
			if (reported_landing > 2) {
				return;
			}

			/* If the rocket isn't on the pad, then report height */
			if (Altos.ao_flight_drogue <= state.state &&
			    state.state < Altos.ao_flight_landed &&
			    state.range >= 0)
			{
				voice.speak("Height %d, bearing %d, elevation %d, range %d.\n",
					    (int) (state.height + 0.5),
					    (int) (state.from_pad.bearing + 0.5),
					    (int) (state.elevation + 0.5),
					    (int) (state.range + 0.5));
			} else if (state.state > Altos.ao_flight_pad) {
				voice.speak("%d meters", (int) (state.height + 0.5));
			} else {
				reported_landing = 0;
			}

			/* If the rocket is coming down, check to see if it has landed;
			 * either we've got a landed report or we haven't heard from it in
			 * a long time
			 */
			if (state.state >= Altos.ao_flight_drogue &&
			    (last ||
			     System.currentTimeMillis() - state.report_time >= 15000 ||
			     state.state == Altos.ao_flight_landed))
			{
				if (Math.abs(state.baro_speed) < 20 && state.height < 100)
					voice.speak("rocket landed safely");
				else
					voice.speak("rocket may have crashed");
				if (state.from_pad != null)
					voice.speak("Bearing %d degrees, range %d meters.",
						    (int) (state.from_pad.bearing + 0.5),
						    (int) (state.from_pad.distance + 0.5));
				++reported_landing;
			}
		}

		long now () {
			return System.currentTimeMillis();
		}

		void set_report_time() {
			report_time = now() + report_interval;
		}

		public void run () {

			reported_landing = 0;
			state = null;
			report_interval = 10000;
			try {
				for (;;) {
					set_report_time();
					for (;;) {
						voice.drain();
						synchronized (this) {
							long	sleep_time = report_time - now();
							if (sleep_time <= 0)
								break;
							wait(sleep_time);
						}
					}
					report(false);
				}
			} catch (InterruptedException ie) {
				try {
					voice.drain();
				} catch (InterruptedException iie) { }
			}
		}

		public synchronized void notice(AltosState new_state, boolean spoken) {
			AltosState old_state = state;
			state = new_state;
			if (!started && state.state > Altos.ao_flight_pad) {
				started = true;
				start();
			}

			if (state.state < Altos.ao_flight_drogue)
				report_interval = 10000;
			else
				report_interval = 20000;
			if (old_state != null && old_state.state != state.state) {
				report_time = now();
				this.notify();
			} else if (spoken)
				set_report_time();
		}
	}

	private boolean tell(AltosState state, AltosState old_state) {
		boolean	ret = false;
		if (old_state == null || old_state.state != state.state) {
			voice.speak(state.data.state());
			if ((old_state == null || old_state.state <= Altos.ao_flight_boost) &&
			    state.state > Altos.ao_flight_boost) {
				voice.speak("max speed: %d meters per second.",
					    (int) (state.max_speed + 0.5));
				ret = true;
			} else if ((old_state == null || old_state.state < Altos.ao_flight_drogue) &&
				   state.state >= Altos.ao_flight_drogue) {
				voice.speak("max height: %d meters.",
					    (int) (state.max_height + 0.5));
				ret = true;
			}
		}
		if (old_state == null || old_state.gps_ready != state.gps_ready) {
			if (state.gps_ready) {
				voice.speak("GPS ready");
				ret = true;
			}
			else if (old_state != null) {
				voice.speak("GPS lost");
				ret = true;
			}
		}
		old_state = state;
		return ret;
	}

	class DisplayThread extends Thread {
		IdleThread	idle_thread;

		String		name;

		int		crc_errors;

		void init() { }

		AltosRecord read() throws InterruptedException, ParseException, AltosCRCException, IOException { return null; }

		void close(boolean interrupted) { }

		void update(AltosState state) throws InterruptedException { }

		public void run() {
			boolean		interrupted = false;
			String		line;
			AltosState	state = null;
			AltosState	old_state = null;
			boolean		told;

			idle_thread = new IdleThread();

			flightInfo.clear();
			try {
				for (;;) {
					try {
						AltosRecord record = read();
						if (record == null)
							break;
						old_state = state;
						state = new AltosState(record, state);
						update(state);
						show(state, crc_errors);
						told = tell(state, old_state);
						idle_thread.notice(state, told);
					} catch (ParseException pp) {
						System.out.printf("Parse error: %d \"%s\"\n", pp.getErrorOffset(), pp.getMessage());
					} catch (AltosCRCException ce) {
						++crc_errors;
						show(state, crc_errors);
					}
				}
			} catch (InterruptedException ee) {
				interrupted = true;
			} catch (IOException ie) {
				JOptionPane.showMessageDialog(AltosUI.this,
							      String.format("Error reading from \"%s\"", name),
							      "Telemetry Read Error",
							      JOptionPane.ERROR_MESSAGE);
			} finally {
				close(interrupted);
				idle_thread.interrupt();
				try {
					idle_thread.join();
				} catch (InterruptedException ie) {}
			}
		}

		public void report() {
			if (idle_thread != null)
				idle_thread.report(true);
		}
	}

	class DeviceThread extends DisplayThread {
		AltosSerial	serial;
		LinkedBlockingQueue<AltosLine> telem;

		AltosRecord read() throws InterruptedException, ParseException, AltosCRCException, IOException {
			AltosLine l = telem.take();
			if (l.line == null)
				throw new IOException("IO error");
			return new AltosTelemetry(l.line);
		}

		void close(boolean interrupted) {
			serial.close();
			serial.remove_monitor(telem);
		}

		public DeviceThread(AltosSerial s, String in_name) {
			serial = s;
			telem = new LinkedBlockingQueue<AltosLine>();
			serial.add_monitor(telem);
			name = in_name;
		}
	}

	private void ConnectToDevice() {
		AltosDevice	device = AltosDeviceDialog.show(AltosUI.this,
								AltosDevice.product_basestation);

		if (device != null) {
			try {
				stop_display();
				serial_line.open(device);
				DeviceThread thread = new DeviceThread(serial_line, device.getPath());
				serial_line.set_channel(AltosPreferences.channel());
				serial_line.set_callsign(AltosPreferences.callsign());
				run_display(thread);
			} catch (FileNotFoundException ee) {
				JOptionPane.showMessageDialog(AltosUI.this,
							      String.format("Cannot open device \"%s\"",
									    device.getPath()),
							      "Cannot open target device",
							      JOptionPane.ERROR_MESSAGE);
			} catch (IOException ee) {
				JOptionPane.showMessageDialog(AltosUI.this,
							      device.getPath(),
							      "Unkonwn I/O error",
							      JOptionPane.ERROR_MESSAGE);
			}
		}
	}

	void DisconnectFromDevice () {
		stop_display();
	}

	void ConfigureCallsign() {
		String	result;
		result = JOptionPane.showInputDialog(AltosUI.this,
						     "Configure Callsign",
						     AltosPreferences.callsign());
		if (result != null) {
			AltosPreferences.set_callsign(result);
			if (serial_line != null)
				serial_line.set_callsign(result);
		}
	}

	void ConfigureTeleMetrum() {
		new AltosConfig(AltosUI.this);
	}

	void FlashImage() {
		new AltosFlashUI(AltosUI.this);
	}

	/*
	 * Open an existing telemetry file and replay it in realtime
	 */

	class ReplayThread extends DisplayThread {
		AltosReader	reader;
		String		name;

		public AltosRecord read() {
			try {
				return reader.read();
			} catch (IOException ie) {
				JOptionPane.showMessageDialog(AltosUI.this,
							      name,
							      "error reading",
							      JOptionPane.ERROR_MESSAGE);
			} catch (ParseException pe) {
			}
			return null;
		}

		public void close (boolean interrupted) {
			if (!interrupted)
				report();
		}

		public ReplayThread(AltosReader in_reader, String in_name) {
			reader = in_reader;
		}
		void update(AltosState state) throws InterruptedException {
			/* Make it run in realtime after the rocket leaves the pad */
			if (state.state > Altos.ao_flight_pad)
				Thread.sleep((int) (Math.min(state.time_change,10) * 1000));
		}
	}

	Thread		display_thread;

	private void stop_display() {
		if (display_thread != null && display_thread.isAlive()) {
			display_thread.interrupt();
			try {
				display_thread.join();
			} catch (InterruptedException ie) {}
		}
		display_thread = null;
	}

	private void run_display(Thread thread) {
		stop_display();
		display_thread = thread;
		display_thread.start();
	}

	/*
	 * Replay a flight from telemetry data
	 */
	private void Replay() {
		AltosLogfileChooser chooser = new AltosLogfileChooser(
			AltosUI.this);
		AltosReader reader = chooser.runDialog();
		if (reader != null)
			run_display(new ReplayThread(reader,
						     chooser.filename()));
	}

	/* Connect to TeleMetrum, either directly or through
	 * a TeleDongle over the packet link
	 */
	private void SaveFlightData() {
		new AltosEepromDownload(AltosUI.this);
	}

	/* Load a flight log file and write out a CSV file containing
	 * all of the data in standard units
	 */

	private void ExportData() {
		new AltosCSVUI(AltosUI.this);
	}

	/* Load a flight log CSV file and display a pretty graph.
	 */

	private void GraphData() {
		new AltosGraphUI(AltosUI.this);
	}

	/* Create the AltosUI menus
	 */
	private void createMenu() {
		JMenuBar menubar = new JMenuBar();
		JMenu menu;
		JMenuItem item;
		JRadioButtonMenuItem radioitem;

		// File menu
		{
			menu = new JMenu("File");
			menu.setMnemonic(KeyEvent.VK_F);
			menubar.add(menu);

			item = new JMenuItem("Replay File",KeyEvent.VK_R);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						Replay();
					}
				});
			menu.add(item);

			item = new JMenuItem("Save Flight Data",KeyEvent.VK_S);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						SaveFlightData();
					}
				});
			menu.add(item);

			item = new JMenuItem("Flash Image",KeyEvent.VK_F);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						FlashImage();
					}
				});
			menu.add(item);

			item = new JMenuItem("Export Data",KeyEvent.VK_F);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						ExportData();
					}
				});
			menu.add(item);

			item = new JMenuItem("Graph Data",KeyEvent.VK_F);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						GraphData();
					}
				});
			menu.add(item);

			item = new JMenuItem("Quit",KeyEvent.VK_Q);
			item.setAccelerator(KeyStroke.getKeyStroke(KeyEvent.VK_Q,
								   ActionEvent.CTRL_MASK));
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						System.exit(0);
					}
				});
			menu.add(item);
		}

		// Device menu
		{
			menu = new JMenu("Device");
			menu.setMnemonic(KeyEvent.VK_D);
			menubar.add(menu);

			item = new JMenuItem("Connect to Device",KeyEvent.VK_C);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						ConnectToDevice();
					}
				});
			menu.add(item);

			item = new JMenuItem("Disconnect from Device",KeyEvent.VK_D);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						DisconnectFromDevice();
					}
				});
			menu.add(item);

			menu.addSeparator();

			item = new JMenuItem("Set Callsign",KeyEvent.VK_S);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						ConfigureCallsign();
					}
				});

			menu.add(item);

			item = new JMenuItem("Configure TeleMetrum device",KeyEvent.VK_T);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						ConfigureTeleMetrum();
					}
				});

			menu.add(item);
		}
		// Log menu
		{
			menu = new JMenu("Log");
			menu.setMnemonic(KeyEvent.VK_L);
			menubar.add(menu);

			item = new JMenuItem("New Log",KeyEvent.VK_N);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
					}
				});
			menu.add(item);

			item = new JMenuItem("Configure Log",KeyEvent.VK_C);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						AltosPreferences.ConfigureLog();
					}
				});
			menu.add(item);
		}
		// Voice menu
		{
			menu = new JMenu("Voice", true);
			menu.setMnemonic(KeyEvent.VK_V);
			menubar.add(menu);

			radioitem = new JRadioButtonMenuItem("Enable Voice", AltosPreferences.voice());
			radioitem.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						JRadioButtonMenuItem item = (JRadioButtonMenuItem) e.getSource();
						boolean enabled = item.isSelected();
						AltosPreferences.set_voice(enabled);
						if (enabled)
							voice.speak_always("Enable voice.");
						else
							voice.speak_always("Disable voice.");
					}
				});
			menu.add(radioitem);
			item = new JMenuItem("Test Voice",KeyEvent.VK_T);
			item.addActionListener(new ActionListener() {
					public void actionPerformed(ActionEvent e) {
						voice.speak("That's one small step for man; one giant leap for mankind.");
					}
				});
			menu.add(item);
		}

		// Channel menu
		{
			menu = new AltosChannelMenu(AltosPreferences.channel());
			menu.addActionListener(new ActionListener() {
						public void actionPerformed(ActionEvent e) {
							int new_channel = Integer.parseInt(e.getActionCommand());
							AltosPreferences.set_channel(new_channel);
							serial_line.set_channel(new_channel);
						}
				});
			menu.setMnemonic(KeyEvent.VK_C);
			menubar.add(menu);
		}

		this.setJMenuBar(menubar);

	}

	static String replace_extension(String input, String extension) {
		int dot = input.lastIndexOf(".");
		if (dot > 0)
			input = input.substring(0,dot);
		return input.concat(extension);
	}

	static AltosReader open_logfile(String filename) {
		File file = new File (filename);
		try {
			FileInputStream in;

			in = new FileInputStream(file);
			if (filename.endsWith("eeprom"))
				return new AltosEepromReader(in);
			else
				return new AltosTelemetryReader(in);
		} catch (FileNotFoundException fe) {
			System.out.printf("Cannot open '%s'\n", filename);
			return null;
		}
	}

	static AltosCSV open_csv(String filename) {
		File file = new File (filename);
		try {
			return new AltosCSV(file);
		} catch (FileNotFoundException fe) {
			System.out.printf("Cannot open '%s'\n", filename);
			return null;
		}
	}

	static void process_file(String input) {
		String output = replace_extension(input,".csv");
		if (input.equals(output)) {
			System.out.printf("Not processing '%s'\n", input);
			return;
		}
		System.out.printf("Processing \"%s\" to \"%s\"\n", input, output);
		AltosReader reader = open_logfile(input);
		if (reader == null)
			return;
		AltosCSV writer = open_csv(output);
		if (writer == null)
			return;
		writer.write(reader);
		reader.close();
		writer.close();
	}

	public static void main(final String[] args) {

		/* Handle batch-mode */
		if (args.length > 0) {
			for (int i = 0; i < args.length; i++)
				process_file(args[i]);
		} else {
			AltosUI altosui = new AltosUI();
			altosui.setVisible(true);
		}
	}
}
