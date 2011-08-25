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

import gov.nasa.worldwind.Model;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.awt.WorldWindowGLCanvas;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.util.StatusBar;
import gov.nasa.worldwind.view.orbit.BasicOrbitView;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;

import javax.swing.JFrame;
import javax.swing.JSplitPane;

import org.ros.worldwind.coord.LocalCoordinateSystem;
import org.ros.worldwind.marker.RenderableManager;
import org.ros.worldwind.ui.Displays;
import org.ros.worldwind.ui.Selection;
import org.ros.worldwind.ui.elements.GlobalOptionsElement;

public class Adapter implements WindowListener {

	private JFrame frame;
	private WorldWindowGLCanvas canvas;
	private Displays displays;
	private Selection selection;
	private RenderableManager renderableManager;
	private LocalCoordinateSystem localCoordinateSystem;

	public Adapter() {
		Dimension dimension = new Dimension(1400, 800);

		frame = new JFrame();
		frame.setTitle("ROS WorldWind Adapter");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.setSize(dimension);
		frame.setLayout(new BorderLayout(0, 0));

		canvas = new WorldWindowGLCanvas();
		Model model = (Model) WorldWind.createConfigurationComponent(AVKey.MODEL_CLASS_NAME);
		canvas.setModel(model);

		renderableManager = new RenderableManager(this);

		selection = new Selection(this);
		displays = new Displays(this);

		JSplitPane leftSplitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
		leftSplitPane.setDividerLocation(300);
		leftSplitPane.setPreferredSize(dimension);
		leftSplitPane.setLeftComponent(displays);
		leftSplitPane.setRightComponent(canvas);

		JSplitPane rightSplitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT);
		rightSplitPane.setDividerLocation(1100);
		rightSplitPane.setPreferredSize(dimension);
		rightSplitPane.setLeftComponent(leftSplitPane);
		rightSplitPane.setRightComponent(selection);
		frame.add(rightSplitPane, BorderLayout.CENTER);

		StatusBar statusBar = new StatusBar();
		statusBar.setEventSource(canvas);
		frame.add(statusBar, BorderLayout.PAGE_END);

		canvas.setPreferredSize(dimension);

		BasicOrbitView view = new BasicOrbitView();
		String lastView = AdapterProperties.getInstance().getProperty(AdapterProperties.LAST_VIEW);
		if (lastView != null)
			view.restoreState(lastView);
		canvas.setView(view);

		frame.addWindowListener(this);

		frame.pack();
		frame.setVisible(true);

		try {
			Thread.sleep(1000);
			localCoordinateSystem = new LocalCoordinateSystem(canvas.getView().getGlobe());
			localCoordinateSystem.setLocalOrigin(
					new Position(new LatLon(Angle.fromDegrees(Double.parseDouble(AdapterProperties.getInstance().getProperty(
							"globalOptions" + GlobalOptionsElement.ORIGIN_LATITUDE_KEY))), Angle.fromDegrees(Double.parseDouble(AdapterProperties
							.getInstance().getProperty("globalOptions" + GlobalOptionsElement.ORIGIN_LONGITUDE_KEY)))), Double
							.parseDouble(AdapterProperties.getInstance().getProperty("globalOptions" + GlobalOptionsElement.ORIGIN_ELEVATION_KEY))),
					Angle.fromDegrees(Double.parseDouble(AdapterProperties.getInstance().getProperty(
							"globalOptions" + GlobalOptionsElement.ORIGIN_ROTATION_XAXIS_KEY))), Angle.fromDegrees(Double
							.parseDouble(AdapterProperties.getInstance()
									.getProperty("globalOptions" + GlobalOptionsElement.ORIGIN_ROTATION_YAXIS_KEY))), Angle.fromDegrees(Double
							.parseDouble(AdapterProperties.getInstance()
									.getProperty("globalOptions" + GlobalOptionsElement.ORIGIN_ROTATION_ZAXIS_KEY))));
		} catch (Exception e) {
			localCoordinateSystem.setLocalOrigin(new Position(new LatLon(Angle.fromDegrees(0), Angle.fromDegrees(0)), 0));
		}
	}

	public WorldWindowGLCanvas getCanvas() {
		return canvas;
	}

	public Displays getDisplays() {
		return displays;
	}

	public Selection getSelection() {
		return selection;
	}

	public RenderableManager getRenderableManager() {
		return renderableManager;
	}

	public LocalCoordinateSystem getLocalCoordinateSystem() {
		return localCoordinateSystem;
	}

	@Override
	public void windowClosed(WindowEvent e) {
		AdapterProperties.getInstance().setProperty(AdapterProperties.LAST_VIEW, canvas.getView().getRestorableState());
		System.exit(0);
	}

	public static void main(String args[]) {
		new Adapter();
	}

	// nothing to do
	public void windowOpened(WindowEvent e) {
	}

	public void windowClosing(WindowEvent e) {
	}

	public void windowIconified(WindowEvent e) {
	}

	public void windowDeiconified(WindowEvent e) {
	}

	public void windowActivated(WindowEvent e) {
	}

	public void windowDeactivated(WindowEvent e) {
	}
}
