/*******************************************************************************
 * Copyright (c) 2011 Martin Frassl, Michael Lichtenstern
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright notice, this 
 *       list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this 
 *       list of conditions and the following disclaimer in the documentation and/or 
 *       other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY 
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
package org.ros.worldwind;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.Properties;

public class AdapterProperties {
	private static AdapterProperties adapterPropertiesObject = null;

	private String propertiesFileName = "rosworldwind.properties";

	private Properties adapterProperties;

	private AdapterProperties() {
		loadProperties();
	}

	private void loadProperties() {
		adapterProperties = new Properties();
		try {
			FileInputStream in = new FileInputStream(propertiesFileName);
			adapterProperties.load(in);
			in.close();
		} catch (Exception e) {
			System.err.println(propertiesFileName + " could not be loaded!");
		}
	}

	private void storeProperties() {
		try {
			FileOutputStream out = new FileOutputStream(propertiesFileName);
			adapterProperties.store(out, "--- ROS WorldWind Properties ---");
			out.close();
		} catch (Exception e) {
			System.err.println(propertiesFileName + " could not be stored");
		}
	}

	public String getProperty(String key) {
		return adapterProperties.getProperty(key);
	}

	public void setProperty(String key, String value) {
		adapterProperties.setProperty(key, value);
		storeProperties();
	}

	public void removeProperty(String key) {
		adapterProperties.remove(key);
		storeProperties();
	}

	public static AdapterProperties getInstance() {
		if (adapterPropertiesObject == null)
			adapterPropertiesObject = new AdapterProperties();
		return adapterPropertiesObject;
	}

	public static final String LAST_VIEW = "globe.lastView";
}
