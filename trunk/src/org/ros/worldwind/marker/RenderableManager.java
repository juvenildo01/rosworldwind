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
package org.ros.worldwind.marker;

import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.layers.RenderableLayer;
import gov.nasa.worldwind.render.RigidShape;
import gov.nasa.worldwind.render.SurfaceIcon;

import org.ros.worldwind.Adapter;

public class RenderableManager {
	private Adapter adapter;
	private RenderableLayer layer;

	public RenderableManager(Adapter adapter) {
		this.adapter = adapter;

		layer = new RenderableLayer();
		layer.setName("marker layer");

		adapter.getCanvas().getModel().getLayers().add(layer);
	}

	public void addMarker(RenderableMarker marker) {
		layer.addRenderable(marker.getSurfaceIcon());
		layer.addRenderable(marker.getRigidShape());
		layer.addRenderable(marker.getSelectionBox());
		layer.addRenderable(marker.getXAxis());
		layer.addRenderable(marker.getYAxis());
		layer.addRenderable(marker.getZAxis());
		layer.addRenderable(marker.getLine());
		layer.addRenderable(marker.getText());
		layer.addRenderable(marker.getTooltip());
		adapter.getCanvas().redraw();
	}

	public void removeMarker(RenderableMarker marker) {
		layer.removeRenderable(marker.getSurfaceIcon());
		layer.removeRenderable(marker.getRigidShape());
		layer.removeRenderable(marker.getSelectionBox());
		layer.removeRenderable(marker.getXAxis());
		layer.removeRenderable(marker.getYAxis());
		layer.removeRenderable(marker.getZAxis());
		layer.removeRenderable(marker.getLine());
		layer.removeRenderable(marker.getText());
		layer.removeRenderable(marker.getTooltip());
		adapter.getCanvas().redraw();
	}

	public void updateMarkerShape(RenderableMarker marker, RigidShape shapeToRemove) {
		layer.removeRenderable(shapeToRemove);
		layer.addRenderable(marker.getRigidShape());
	}

	private SurfaceIcon originIcon;

	public void addOrigin(LatLon position) {
		originIcon = new SurfaceIcon("gfx/origin.png", position);
		layer.addRenderable(originIcon);
		adapter.getCanvas().redraw();
	}

	public void removeOrigin() {
		if (originIcon != null)
			layer.removeRenderable(originIcon);
		adapter.getCanvas().redraw();
	}

}
