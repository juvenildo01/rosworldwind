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

import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.view.orbit.FlyToOrbitViewAnimator;
import gov.nasa.worldwind.view.orbit.OrbitView;

import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;

import net.miginfocom.swing.MigLayout;

import org.ros.worldwind.Adapter;

public abstract class AbstractElement extends JPanel {
	private static final long serialVersionUID = -545518425694935872L;

	protected String persistenceID;

	protected Adapter adapter;

	public AbstractElement(Adapter adapter, String persistenceID) {
		this.adapter = adapter;
		this.persistenceID = persistenceID;

		setLayout(new MigLayout("hidemode 3, gap 8 2, novisualpadding, ins 4", "[][fill, grow]", "[]"));
	}

	protected void addRow(JLabel label, JComponent component) {
		add(label);
		add(component, "wrap");
	}

	protected void addRow(JComponent component, String layoutConstraints) {
		add(component, "span 2, grow, wrap" + (layoutConstraints.equalsIgnoreCase("") ? "" : ", " + layoutConstraints));
	}

	public abstract void updateOrigin();

	protected void flyTo(Position position, double zoom) {
		OrbitView currentView = (OrbitView) adapter.getCanvas().getView();

		currentView.stopAnimations();
		currentView.addAnimator(FlyToOrbitViewAnimator.createFlyToOrbitViewAnimator(currentView, currentView.getCenterPosition(), position,
				currentView.getHeading(), currentView.getHeading(), currentView.getPitch(), currentView.getPitch(), currentView.getZoom(), zoom,
				2000, WorldWind.RELATIVE_TO_GROUND));
		adapter.getCanvas().redraw();
	}
}
