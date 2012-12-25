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

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.*;
import org.altusmetrum.AltosLib.*;

public class MicroFileChooser extends JFileChooser {
	JFrame	frame;
	String	filename;
	File	file;

	public String filename() {
		return filename;
	}

	public File file() {
		return file;
	}

	public InputStream runDialog() {
		int	ret;

		ret = showOpenDialog(frame);
		if (ret == APPROVE_OPTION) {
			file = getSelectedFile();
			if (file == null)
				return null;
			filename = file.getName();
			try {
				return new FileInputStream(file);
			} catch (FileNotFoundException fe) {
				JOptionPane.showMessageDialog(frame,
							      fe.getMessage(),
							      "Cannot open file",
							      JOptionPane.ERROR_MESSAGE);
			}
		}
		return null;
	}

	public MicroFileChooser(JFrame in_frame) {
		frame = in_frame;
		setDialogTitle("Select MicroPeak Data File");
		setFileFilter(new FileNameExtensionFilter("MicroPeak data file",
							  "mpd"));
	}
}
