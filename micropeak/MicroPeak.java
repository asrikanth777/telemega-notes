/*
 * Copyright © 2012 Keith Packard <keithp@keithp.com>
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

package org.altusmetrum.micropeak;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.util.concurrent.*;
import java.util.*;
import org.altusmetrum.AltosLib.*;
import org.altusmetrum.altosuilib.*;

public class MicroPeak extends MicroFrame implements ActionListener, ItemListener {

	File		filename;
	MicroGraph	graph;
	MicroStatsTable	stats;
	MicroData	data;
	Container	container;
	JTabbedPane	pane;

	private void RunFile(InputStream input) {
		try {
			data = new MicroData(input);
			graph.setData(data);
			stats.setData(data);
		} catch (IOException ioe) {
		}
		try {
			input.close();
		} catch (IOException ioe) {
		}
	}

	private void OpenFile(File filename) {
		try {
			RunFile (new FileInputStream(filename));
		} catch (FileNotFoundException fne) {
		}
	}

	private void SelectFile() {
		MicroFileChooser	chooser = new MicroFileChooser(this);
		InputStream		input = chooser.runDialog();

		if (input != null)
			RunFile(input);
	}

	private void Preferences() {
		new AltosConfigureUI(this);
	}
		
	private void DownloadData() {
		java.util.List<MicroUSB>	devices = MicroUSB.list();
		for (MicroUSB device : devices)
			System.out.printf("device %s\n", device.toString());
	}

	public void actionPerformed(ActionEvent ev) {
		if ("Exit".equals(ev.getActionCommand()))
			System.exit(0);
		else if ("Open".equals(ev.getActionCommand()))
			SelectFile();
		else if ("New".equals(ev.getActionCommand()))
			new MicroPeak();
		else if ("Download".equals(ev.getActionCommand()))
			DownloadData();
		else if ("Preferences".equals(ev.getActionCommand()))
			Preferences();
	}

	public void itemStateChanged(ItemEvent e) {
	}

	public MicroPeak() {

		AltosUIPreferences.set_component(this);

		container = getContentPane();
		pane = new JTabbedPane();

		setTitle("MicroPeak");

		JMenuBar menuBar = new JMenuBar();
		setJMenuBar(menuBar);

		JMenu fileMenu = new JMenu("File");
		menuBar.add(fileMenu);

		JMenuItem newAction = new JMenuItem("New");
		fileMenu.add(newAction);
		newAction.addActionListener(this);

		JMenuItem openAction = new JMenuItem("Open");
		fileMenu.add(openAction);
		openAction.addActionListener(this);

		JMenuItem downloadAction = new JMenuItem("Download");
		fileMenu.add(downloadAction);
		downloadAction.addActionListener(this);

		JMenuItem preferencesAction = new JMenuItem("Preferences");
		fileMenu.add(preferencesAction);
		preferencesAction.addActionListener(this);

		JMenuItem exitAction = new JMenuItem("Exit");
		fileMenu.add(exitAction);
		exitAction.addActionListener(this);

		setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE);
		addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});

		graph = new MicroGraph();
		stats = new MicroStatsTable();
		pane.add(graph.panel, "Graph");
		pane.add(stats, "Statistics");
		pane.doLayout();
		pane.validate();
		container.add(pane);
		container.doLayout();
		container.validate();
		doLayout();
		validate();
		Insets i = getInsets();
		Dimension ps = pane.getPreferredSize();
		ps.width += i.left + i.right;
		ps.height += i.top + i.bottom;
		setPreferredSize(ps);
		setSize(ps);
		setVisible(true);
	}

	public static void main(final String[] args) {
		boolean	opened = false;

		try {
			UIManager.setLookAndFeel(AltosUIPreferences.look_and_feel());
		} catch (Exception e) {
		}

		for (int i = 0; i < args.length; i++) {
			MicroPeak m = new MicroPeak();
			m.OpenFile(new File(args[i]));
			opened = true;
		}
		if (!opened)
			new MicroPeak();
	}
}