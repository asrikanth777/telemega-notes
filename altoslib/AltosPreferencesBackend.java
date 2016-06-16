/*
 * Copyright © 2010 Mike Beattie <mike@ethernal.org>
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

package org.altusmetrum.altoslib_11;

import java.io.*;
import java.util.*;
import java.text.*;

public abstract class AltosPreferencesBackend {

	public abstract String  getString(String key, String def);
	public abstract void    putString(String key, String value);

	public abstract int     getInt(String key, int def);
	public abstract void    putInt(String key, int value);

	public abstract double  getDouble(String key, double def);
	public abstract void    putDouble(String key, double value);

	public abstract boolean getBoolean(String key, boolean def);
	public abstract void    putBoolean(String key, boolean value);

	public abstract byte[]  getBytes(String key, byte[] def);
	public abstract void    putBytes(String key, byte[] value);

	public AltosJson	getJson(String key) {
		String	value = getString(key, null);

		if (value == null)
			return null;
		try {
			return AltosJson.fromString(value);
		} catch (IllegalArgumentException ie) {
			return null;
		}
	}

	public void	       	putJson(String key, AltosJson j) {
		putString(key, j.toString());
	}

	public abstract boolean nodeExists(String key);
	public abstract AltosPreferencesBackend node(String key);

	public abstract String[] keys();
	public abstract void    remove(String key);

	public abstract void    flush();

	public abstract File homeDirectory();

	public abstract void debug(String format, Object ... arguments);
}
