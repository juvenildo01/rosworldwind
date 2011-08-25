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
package org.ros.worldwind.ui.elements;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;

import java.awt.Cursor;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.text.NumberFormat;

import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JFormattedTextField;
import javax.swing.JLabel;

import org.ros.worldwind.Adapter;
import org.ros.worldwind.AdapterProperties;
import org.ros.worldwind.ui.Displays;

public class GlobalOptionsElement extends AbstractElement {
	private static final long serialVersionUID = 5198594413987348378L;

	public static final String ORIGIN_LATITUDE_KEY = ".origin.latitude";
	public static final String ORIGIN_LONGITUDE_KEY = ".origin.longitude";
	public static final String ORIGIN_ELEVATION_KEY = ".origin.elevation";
	public static final String ORIGIN_ROTATION_XAXIS_KEY = ".origin.rotation.xaxis";
	public static final String ORIGIN_ROTATION_YAXIS_KEY = ".origin.rotation.yaxis";
	public static final String ORIGIN_ROTATION_ZAXIS_KEY = ".origin.rotation.zaxis";
	public static final String ORIGIN_SHOW_KEY = ".origin.show";

	private OriginMouseAdapter originMouseAdapter;

	private NumberFormat latlonFormat;
	private NumberFormat meterFormat;
	private NumberFormat rotationFormat;

	private JFormattedTextField latitude;
	private JFormattedTextField longitude;
	private JFormattedTextField elevation;
	private JFormattedTextField rotationXAxis;
	private JFormattedTextField rotationYAxis;
	private JFormattedTextField rotationZAxis;
	private JCheckBox showOrigin;

	public GlobalOptionsElement(Adapter adapter, String persistenceID) {
		super(adapter, persistenceID);

		originMouseAdapter = new OriginMouseAdapter();

		latlonFormat = NumberFormat.getNumberInstance();
		latlonFormat.setMinimumFractionDigits(5);
		meterFormat = NumberFormat.getNumberInstance();
		meterFormat.setMaximumFractionDigits(2);
		rotationFormat = NumberFormat.getNumberInstance();
		rotationFormat.setMinimumFractionDigits(1);
		rotationFormat.setMaximumFractionDigits(1);

		TextAreaActionListener textAreaActionListener = new TextAreaActionListener();

		latitude = new JFormattedTextField(latlonFormat);
		latitude.setValue(readDouble(AdapterProperties.getInstance().getProperty(persistenceID + ORIGIN_LATITUDE_KEY)));
		latitude.addActionListener(textAreaActionListener);
		addRow(new JLabel("latitude (°)"), latitude);

		longitude = new JFormattedTextField(latlonFormat);
		longitude.setValue(readDouble(AdapterProperties.getInstance().getProperty(persistenceID + ORIGIN_LONGITUDE_KEY)));
		longitude.addActionListener(textAreaActionListener);
		addRow(new JLabel("longitude (°)"), longitude);

		elevation = new JFormattedTextField(meterFormat);
		elevation.setValue(readDouble(AdapterProperties.getInstance().getProperty(persistenceID + ORIGIN_ELEVATION_KEY)));
		elevation.addActionListener(textAreaActionListener);
		addRow(new JLabel("elevation (m)"), elevation);

		rotationXAxis = new JFormattedTextField(rotationFormat);
		rotationXAxis.setValue(readDouble(AdapterProperties.getInstance().getProperty(persistenceID + ORIGIN_ROTATION_XAXIS_KEY)));
		rotationXAxis.addActionListener(textAreaActionListener);
		addRow(new JLabel("rotation x (°)"), rotationXAxis);

		rotationYAxis = new JFormattedTextField(rotationFormat);
		rotationYAxis.setValue(readDouble(AdapterProperties.getInstance().getProperty(persistenceID + ORIGIN_ROTATION_YAXIS_KEY)));
		rotationYAxis.addActionListener(textAreaActionListener);
		addRow(new JLabel("rotation y (°)"), rotationYAxis);

		rotationZAxis = new JFormattedTextField(rotationFormat);
		rotationZAxis.setValue(readDouble(AdapterProperties.getInstance().getProperty(persistenceID + ORIGIN_ROTATION_ZAXIS_KEY)));
		rotationZAxis.addActionListener(textAreaActionListener);
		addRow(new JLabel("rotation z (°)"), rotationZAxis);

		showOrigin = new JCheckBox();
		showOrigin.setToolTipText("show origin on globe");
		showOrigin.addItemListener(new OriginItemListener());
		showOrigin.setSelected(Boolean.parseBoolean(AdapterProperties.getInstance().getProperty(persistenceID + ORIGIN_SHOW_KEY)));
		showOrigin.setOpaque(false);
		addRow(new JLabel("show"), showOrigin);

		addRow(initFlyToOriginButton(), Displays.LAYOUT_BUTTON);

		addRow(initSetOriginButton(), Displays.LAYOUT_BUTTON);
	}

	private double readDouble(String doubleString) {
		try {
			return Double.parseDouble(doubleString);
		} catch (Exception e) {
		}
		return 0;
	}

	private JButton initFlyToOriginButton() {
		JButton flyToOrigin = new JButton("fly to origin");
		flyToOrigin.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				flyTo(generateOriginPositionFromProperties(), 50000);
			}
		});
		return flyToOrigin;
	}

	private JButton initSetOriginButton() {
		JButton setOrigin = new JButton("set origin");
		setOrigin.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				adapter.getCanvas().getInputHandler().addMouseListener(originMouseAdapter);
				adapter.getCanvas().setCursor(Cursor.getPredefinedCursor(Cursor.CROSSHAIR_CURSOR));
			}
		});
		return setOrigin;
	}

	private void storeOrigin() {
		AdapterProperties.getInstance().setProperty(persistenceID + ORIGIN_LATITUDE_KEY, latitude.getText().replace(",", ""));
		AdapterProperties.getInstance().setProperty(persistenceID + ORIGIN_LONGITUDE_KEY, longitude.getText().replace(",", ""));
		AdapterProperties.getInstance().setProperty(persistenceID + ORIGIN_ELEVATION_KEY, elevation.getText().replace(",", ""));
		AdapterProperties.getInstance().setProperty(persistenceID + ORIGIN_ROTATION_XAXIS_KEY, rotationXAxis.getText().replace(",", ""));
		AdapterProperties.getInstance().setProperty(persistenceID + ORIGIN_ROTATION_YAXIS_KEY, rotationYAxis.getText().replace(",", ""));
		AdapterProperties.getInstance().setProperty(persistenceID + ORIGIN_ROTATION_ZAXIS_KEY, rotationZAxis.getText().replace(",", ""));
	}

	private void renderOrigin() {
		adapter.getRenderableManager().removeOrigin();
		if (showOrigin.isSelected())
			adapter.getRenderableManager().addOrigin(generateOriginLatLonFromProperties());
	}

	@Override
	public void updateOrigin() {
		renderOrigin();
	}

	private LatLon generateOriginLatLonFromProperties() {
		return new LatLon(Angle.fromDegrees(Double.parseDouble(latitude.getText())), Angle.fromDegrees(Double.parseDouble(longitude.getText()
				.replace(",", ""))));
	}

	private Position generateOriginPositionFromProperties() {
		return new Position(generateOriginLatLonFromProperties(), Double.parseDouble(elevation.getText().replace(",", "")));
	}

	public void updateDisplay() {
		Position origin = generateOriginPositionFromProperties();
		adapter.getLocalCoordinateSystem().setLocalOrigin(origin, Angle.fromDegrees(Double.parseDouble(rotationXAxis.getText().replace(",", ""))),
				Angle.fromDegrees(Double.parseDouble(rotationYAxis.getText().replace(",", ""))),
				Angle.fromDegrees(Double.parseDouble(rotationZAxis.getText().replace(",", ""))));
		adapter.getDisplays().updateOrigin();
		storeOrigin();
	}

	private class OriginMouseAdapter extends MouseAdapter {
		public void mouseClicked(MouseEvent mouseEvent) {
			if (mouseEvent.getButton() == MouseEvent.BUTTON1) {
				Position newOriginPosition = adapter.getCanvas().getCurrentPosition();
				latitude.setText(newOriginPosition.getLatitude().getDegrees() + "");
				longitude.setText(newOriginPosition.getLongitude().getDegrees() + "");
				adapter.getCanvas().setCursor(Cursor.getPredefinedCursor(Cursor.DEFAULT_CURSOR));
				adapter.getCanvas().getInputHandler().removeMouseListener(originMouseAdapter);
				updateDisplay();
			}
			// prevents that the globe is turning on mouse click
			mouseEvent.consume();
		}
	}

	private class TextAreaActionListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			updateDisplay();
		}
	}

	private class OriginItemListener implements ItemListener {
		@Override
		public void itemStateChanged(ItemEvent e) {
			if (e != null) {
				renderOrigin();
				AdapterProperties.getInstance().setProperty(persistenceID + ORIGIN_SHOW_KEY, showOrigin.isSelected() + "");
			}
		}
	}

}
