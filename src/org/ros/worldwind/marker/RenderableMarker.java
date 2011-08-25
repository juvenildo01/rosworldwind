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

import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.event.SelectEvent;
import gov.nasa.worldwind.event.SelectListener;
import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.LatLon;
import gov.nasa.worldwind.geom.Matrix;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Quaternion;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.render.AnnotationAttributes;
import gov.nasa.worldwind.render.BasicShapeAttributes;
import gov.nasa.worldwind.render.Box;
import gov.nasa.worldwind.render.Cylinder;
import gov.nasa.worldwind.render.DrawContext;
import gov.nasa.worldwind.render.Ellipsoid;
import gov.nasa.worldwind.render.GlobeAnnotation;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Path;
import gov.nasa.worldwind.render.RigidShape;
import gov.nasa.worldwind.render.ScreenAnnotation;
import gov.nasa.worldwind.render.ShapeAttributes;
import gov.nasa.worldwind.render.SurfaceIcon;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Insets;
import java.awt.Rectangle;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Vector;

import org.ros.DefaultNode;
import org.ros.MessageListener;
import org.ros.Node;
import org.ros.NodeConfiguration;
import org.ros.NodeMain;
import org.ros.message.geometry_msgs.Point;
import org.ros.message.visualization_msgs.Marker;
import org.ros.worldwind.Adapter;
import org.ros.worldwind.ui.elements.MarkerElement;

public class RenderableMarker {
	private Adapter adapter;
	private MarkerElement parent;
	private String topicName;
	private MarkerSubscriber markerSubscriber;

	private Marker marker;

	private SurfaceIcon surfaceIcon;
	private String imagePath;
	private RigidShape rigidShape;
	private ShapeAttributes selectionBoxAttributes;
	private Box selectionBox;
	private ShapeAttributes rigidShapeAttributes;
	private Box xAxis;
	private ShapeAttributes xAxisAttributes;
	private Box yAxis;
	private ShapeAttributes yAxisAttributes;
	private Box zAxis;
	private ShapeAttributes zAxisAttributes;
	private GlobeAnnotation text;
	private AnnotationAttributes textAttributes;
	private Path line;
	private ShapeAttributes lineAttributes;

	private MarkerSelectListener markerSelectListener;
	private Tooltip tooltip;
	private boolean showTooltip;

	public RenderableMarker(String topicName, Adapter adapter, MarkerElement parent) {
		this.topicName = topicName;
		this.adapter = adapter;
		this.parent = parent;

		markerSubscriber = new MarkerSubscriber(new MarkerMessageListener());
		subscribe(topicName);

		adapter.getCanvas().getInputHandler().addMouseListener(new MarkerMouseAdapter());
		markerSelectListener = new MarkerSelectListener();
		adapter.getCanvas().getInputHandler().addSelectListener(markerSelectListener);

		rigidShapeAttributes = new BasicShapeAttributes();
		rigidShapeAttributes.setInteriorMaterial(Material.ORANGE);
		rigidShapeAttributes.setInteriorOpacity(0.6);
		rigidShapeAttributes.setDrawOutline(false);
		rigidShapeAttributes.setEnableLighting(true);
		rigidShape = new Ellipsoid();
		rigidShape.setAttributes(rigidShapeAttributes);

		selectionBoxAttributes = new BasicShapeAttributes();
		selectionBoxAttributes.setInteriorMaterial(Material.WHITE);
		selectionBoxAttributes.setInteriorOpacity(0);
		selectionBoxAttributes.setOutlineWidth(2);
		selectionBoxAttributes.setOutlineMaterial(Material.CYAN);
		selectionBoxAttributes.setDrawOutline(true);
		selectionBox = new Box();
		selectionBox.setAttributes(selectionBoxAttributes);
		setSelected(false);

		xAxisAttributes = new BasicShapeAttributes();
		xAxisAttributes.setInteriorMaterial(Material.RED);
		xAxisAttributes.setInteriorOpacity(0.4);
		xAxisAttributes.setDrawOutline(false);
		xAxis = new Box();
		xAxis.setAttributes(xAxisAttributes);

		yAxisAttributes = new BasicShapeAttributes();
		yAxisAttributes.setInteriorMaterial(Material.GREEN);
		yAxisAttributes.setInteriorOpacity(0.4);
		yAxisAttributes.setDrawOutline(false);
		yAxis = new Box();
		yAxis.setAttributes(yAxisAttributes);
		yAxis.setRoll(Angle.fromDegrees(0));
		yAxis.setHeading(Angle.fromDegrees(0));
		yAxis.setTilt(Angle.fromDegrees(0));

		zAxisAttributes = new BasicShapeAttributes();
		zAxisAttributes.setInteriorMaterial(Material.BLUE);
		zAxisAttributes.setInteriorOpacity(0.4);
		zAxisAttributes.setDrawOutline(false);
		zAxis = new Box();
		zAxis.setAttributes(zAxisAttributes);

		textAttributes = new AnnotationAttributes();
		textAttributes.setCornerRadius(10);
		textAttributes.setBackgroundColor(new Color(1.0f, 1.0f, 1.0f, 0.0f));
		textAttributes.setTextColor(new Color(1.0f, 1.0f, 1.0f, 1.0f));

		textAttributes.setBorderWidth(0);
		textAttributes.setDrawOffset(new java.awt.Point(0, 0));
		textAttributes.setImageRepeat(AVKey.REPEAT_NONE);
		text = new GlobeAnnotation("", Position.ZERO, textAttributes);

		lineAttributes = new BasicShapeAttributes();
		lineAttributes.setOutlineMaterial(Material.WHITE);
		lineAttributes.setOutlineOpacity(0.6);
		lineAttributes.setOutlineWidth(2);
		line = new Path();
		line.setAttributes(lineAttributes);

		surfaceIcon = new SurfaceIcon("");
		setSurfaceImagePath("gfx/dummy.png");

		tooltip = new Tooltip("", new java.awt.Point(0, 0));
		setShowTooltip(false);

		setVisible(VISIBLE_NONE);
	}

	public void setTopicName(String topicName) {
		this.topicName = topicName;
	}

	public Marker getMarker() {
		return marker;
	}

	public void subscribe(String topicName) {
		if (topicName != null && !topicName.equalsIgnoreCase("")) {
			setTopicName(topicName);
			markerSubscriber.main(NodeConfiguration.createDefault());
		}
	}

	public SurfaceIcon getSurfaceIcon() {
		return surfaceIcon;
	}

	public RigidShape getRigidShape() {
		return rigidShape;
	}

	public Box getSelectionBox() {
		return selectionBox;
	}

	public Box getXAxis() {
		return xAxis;
	}

	public Box getYAxis() {
		return yAxis;
	}

	public Box getZAxis() {
		return zAxis;
	}

	public GlobeAnnotation getText() {
		return text;
	}

	public ScreenAnnotation getTooltip() {
		return tooltip;
	}

	public Path getLine() {
		return line;
	}

	public void setSelected(boolean selected) {
		selectionBox.setVisible(selected);
	}

	public boolean isShowTooltip() {
		return showTooltip;
	}

	public void setShowTooltip(boolean showTooltip) {
		this.showTooltip = showTooltip;
	}

	public void setParent(MarkerElement parent) {
		this.parent = parent;
	}

	private double checkForPositive(double value) {
		return value > 0 ? value : 0.00000000001;
	}

	public void setScale(double x, double y, double z) {
		x = checkForPositive(x);
		y = checkForPositive(y);
		z = checkForPositive(z);

		selectionBox.setEastWestRadius(x);
		selectionBox.setNorthSouthRadius(y);
		selectionBox.setVerticalRadius(z);

		x = x / 2;
		y = y / 2;
		z = z / 2;

		rigidShape.setEastWestRadius(x);
		rigidShape.setNorthSouthRadius(y);
		rigidShape.setVerticalRadius(z);

		double decreaseFactor = 50;
		double increaseFactor = 2.8;

		xAxis.setEastWestRadius(x * increaseFactor);
		xAxis.setNorthSouthRadius(y / decreaseFactor);
		xAxis.setVerticalRadius(z / decreaseFactor);

		yAxis.setEastWestRadius(x / decreaseFactor);
		yAxis.setNorthSouthRadius(y * increaseFactor);
		yAxis.setVerticalRadius(z / decreaseFactor);

		zAxis.setNorthSouthRadius(x / decreaseFactor);
		zAxis.setEastWestRadius(y / decreaseFactor);
		zAxis.setVerticalRadius(z * increaseFactor);
	}

	public double getScale() {
		return selectionBox.getNorthSouthRadius();
	}

	public void setColor(float r, float g, float b, float a) {
		Color color = new Color(r, g, b, a);
		rigidShapeAttributes.setInteriorMaterial(new Material(color));
		textAttributes.setTextColor(color);
		lineAttributes.setOutlineMaterial(new Material(color));
	}

	public void setLocation(LatLon location, double elevation) {
		setLocation(new Position(location, elevation));
	}

	private Position convertLocation(Point point) {
		return adapter.getLocalCoordinateSystem().transform(new Vec4(point.x, point.y, point.z));
	}

	public Position getPosition() {
		return rigidShape.getCenterPosition();
	}

	public void setLocation(Position location) {
		surfaceIcon.setLocation(location);
		text.setPosition(location);
		rigidShape.setCenterPosition(location);
		selectionBox.setCenterPosition(location);
		xAxis.setCenterPosition(location);
		yAxis.setCenterPosition(location);
		zAxis.setCenterPosition(location);
	}

	public void setLocations(ArrayList<Point> points) {
		if (points != null && !points.isEmpty()) {
			Vector<Position> positions = new Vector<Position>();
			for (Point point : points) {
				positions.add(convertLocation(point));
			}
			line.setPositions(positions);
		}
	}

	public void setHeading(Angle heading) {

		surfaceIcon.setHeading(heading);
		rigidShape.setHeading(heading);
		selectionBox.setHeading(heading);
		xAxis.setHeading(heading);
		yAxis.setHeading(heading);
		zAxis.setHeading(heading);
	}

	public void setYaw(Angle yaw) {
		setHeading(yaw);
	}

	public void setTilt(Angle roll) {

		rigidShape.setRoll(roll);
		selectionBox.setRoll(roll);
		xAxis.setRoll(roll);
		yAxis.setRoll(roll);
		zAxis.setRoll(roll);
	}

	public void setRoll(Angle tilt) {

		rigidShape.setTilt(tilt);
		selectionBox.setTilt(tilt);
		xAxis.setTilt(tilt);
		yAxis.setTilt(tilt);
		zAxis.setTilt(tilt);
	}

	public void setRollAndTilt(Angle roll, Angle tilt) {
		setRoll(roll);
		setTilt(tilt);
	}

	public void setSurfaceImagePath(String path) {
		imagePath = path;
		surfaceIcon.setImageSource(imagePath);
		ArrayList<Object> imageSources = new ArrayList<Object>();
		imageSources.add(null);
		if (imagePath != null && !imagePath.equalsIgnoreCase(""))
			imageSources.add(imagePath);
		rigidShape.setImageSources(imageSources);
		adapter.getCanvas().redraw();
	}

	private static int VISIBLE_NONE = -1;
	private static int VISIBLE_SHAPE = 0;
	private static int VISIBLE_LINE = 1;
	private static int VISIBLE_TEXT = 2;

	private void setVisible(int visible) {
		rigidShape.setVisible(visible == VISIBLE_SHAPE);
		xAxis.setVisible(visible == VISIBLE_SHAPE);
		yAxis.setVisible(visible == VISIBLE_SHAPE);
		zAxis.setVisible(visible == VISIBLE_SHAPE);
		line.setVisible(visible == VISIBLE_LINE);
		text.getAttributes().setVisible(visible == VISIBLE_TEXT);
	}

	public void update() {
		java.awt.EventQueue.invokeLater(new Runnable() {
			public void run() {
				if (marker != null && adapter.getLocalCoordinateSystem() != null) {
					selectType(marker.type);
					setLocation(convertLocation(marker.pose.position));
					Quaternion quaternion = new Quaternion(marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z,
							marker.pose.orientation.w);

					Matrix M = Matrix.IDENTITY;
					M = M.multiply(adapter.getLocalCoordinateSystem().getRotation());
					Matrix M4 = M.multiply(Matrix.fromQuaternion(quaternion));
					setYaw(Angle.fromDegrees(M4.getRotationZ().degrees));
					setRoll(Angle.fromDegrees(M4.getRotationX().degrees));
					setTilt(Angle.fromDegrees(M4.getRotationY().degrees));

					setScale(marker.scale.x, marker.scale.y, marker.scale.z);
					setColor(marker.color.r, marker.color.g, marker.color.b, marker.color.a);
					setLocations(marker.points);
					text.setText(marker.text);

					if (parent != null)
						parent.updateSelectionPanel();

					markerSelectListener.renderText(null);
				}
			}
		});
		adapter.getCanvas().redraw();
	}

	private void selectType(int type) {
		switch (type) {
		case 1:
			if (!(rigidShape instanceof Box)) {
				RigidShape shapeToRemove = rigidShape;
				rigidShape = new Box();
				rigidShape.setAttributes(rigidShapeAttributes);
				adapter.getRenderableManager().updateMarkerShape(this, shapeToRemove);
				setSurfaceImagePath(imagePath);
				setVisible(VISIBLE_SHAPE);
			}
			break;
		case 2:
			if (!(rigidShape instanceof Ellipsoid)) {
				RigidShape shapeToRemove = rigidShape;
				rigidShape = new Ellipsoid();
				rigidShape.setAttributes(rigidShapeAttributes);
				adapter.getRenderableManager().updateMarkerShape(this, shapeToRemove);
				setVisible(VISIBLE_SHAPE);
			}
			break;
		case 3:
			if (!(rigidShape instanceof Cylinder)) {
				RigidShape shapeToRemove = rigidShape;
				rigidShape = new Cylinder();
				rigidShape.setAttributes(rigidShapeAttributes);
				adapter.getRenderableManager().updateMarkerShape(this, shapeToRemove);
				setVisible(VISIBLE_SHAPE);
			}
			break;
		case 4:
			if (!line.isVisible()) {
				setVisible(VISIBLE_LINE);
			}
			break;
		case 9:
			if (!text.getAttributes().isVisible()) {
				setVisible(VISIBLE_TEXT);
			}
			break;
		}
	}

	private class MarkerMessageListener implements MessageListener<org.ros.message.visualization_msgs.Marker> {
		@Override
		public void onNewMessage(org.ros.message.visualization_msgs.Marker message) {
			if (message != null) {
				marker = message;
				update();
			}
		}
	}

	private class MarkerSubscriber implements NodeMain {
		private Node node;
		private MessageListener<Marker> listener;

		public MarkerSubscriber(MessageListener<Marker> listener) {
			setTopicName(topicName);
			setListener(listener);
		}

		@Override
		public void main(NodeConfiguration configuration) {
			try {
				node = new DefaultNode(topicName + System.currentTimeMillis(), configuration);
				node.createSubscriber(topicName, "visualization_msgs/Marker", listener);
			} catch (Exception e) {
				if (node != null) {
					node.getLog().fatal(e);
				} else {
					e.printStackTrace();
				}
			}
		}

		@Override
		public void shutdown() {
			if (node != null)
				node.shutdown();
		}

		private void setListener(MessageListener<Marker> listener) {
			this.listener = listener;
		}
	}

	private class MarkerMouseAdapter extends MouseAdapter {
		public void mouseClicked(MouseEvent mouseEvent) {
			if (mouseEvent == null || mouseEvent.isConsumed()) {
				return;
			}
			if (mouseEvent.getButton() == MouseEvent.BUTTON1) {
				Object topObject = adapter.getCanvas().getObjectsAtCurrentPosition().getTopObject();
				if (!(topObject == rigidShape || topObject == selectionBox))
					return;
				else if (parent != null)
					parent.setSelection(!parent.isSelection());
			}
			// prevents that the globe is turning on mouse click
			mouseEvent.consume();
		}
	}

	private class MarkerSelectListener implements SelectListener {
		private NumberFormat numberFormat;
		private String text = "";
		private boolean isVisible = false;

		public MarkerSelectListener() {
			numberFormat = NumberFormat.getNumberInstance();
			numberFormat.setMaximumFractionDigits(4);
			numberFormat.setMaximumFractionDigits(4);
		}

		@Override
		public void selected(SelectEvent event) {
			if (event.getEventAction().equals(SelectEvent.HOVER) && isShowTooltip()) {
				if (event.getTopObject() != null) {
					Object topObject = event.getObjects().getTopObject();
					if (topObject != null) {
						if (topObject == rigidShape || topObject == selectionBox) {
							isVisible = true;
							renderText(event.getPickPoint());
							return;
						}
					}
				}
			}
			isVisible = false;
			renderText(null);
		}

		public void renderText(java.awt.Point point) {
			if (isVisible) {
				text = topicName + " | ID: " + marker.id + " | seq: " + marker.header.seq + "\n";
				text += "position | x: " + numberFormat.format(marker.pose.position.x) + " y: " + numberFormat.format(marker.pose.position.y)
						+ " z: " + numberFormat.format(marker.pose.position.z) + "\n";
				text += "rotation | x: " + numberFormat.format(marker.pose.orientation.x) + " y: " + numberFormat.format(marker.pose.orientation.y)
						+ " z: " + numberFormat.format(marker.pose.orientation.z) + " w: " + numberFormat.format(marker.pose.orientation.w) + "\n";
				text += "scale | x: " + numberFormat.format(marker.scale.x) + " y: " + numberFormat.format(marker.scale.y) + " z: "
						+ numberFormat.format(marker.scale.z);
			}
			tooltip.showTooltip(isVisible, text, point);
		}
	}

	// we reused the class gov.nasa.worldwindx.examples.util.ToolTipAnnotation
	// as this class is not included into the jar file
	private class Tooltip extends ScreenAnnotation {

		private AnnotationAttributes tooltipAnnotationAttributes;

		public Tooltip(String text, java.awt.Point position) {
			super(text, position);
			tooltipAnnotationAttributes = new AnnotationAttributes();
			tooltipAnnotationAttributes.setAdjustWidthToText(AVKey.SIZE_FIT_TEXT);
			tooltipAnnotationAttributes.setFrameShape(AVKey.SHAPE_NONE);
			tooltipAnnotationAttributes.setTextColor(Color.WHITE);
			tooltipAnnotationAttributes.setBackgroundColor(new Color(0f, 0f, 0f, 0.1f));
			tooltipAnnotationAttributes.setCornerRadius(5);
			tooltipAnnotationAttributes.setBorderColor(new Color(120, 120, 120));
			tooltipAnnotationAttributes.setTextAlign(AVKey.LEFT);
			tooltipAnnotationAttributes.setInsets(new Insets(10, 10, 10, 10));
			tooltipAnnotationAttributes.setSize(new Dimension(800, 0));
			tooltipAnnotationAttributes.setVisible(false);

			setAttributes(tooltipAnnotationAttributes);
			setPickEnabled(false);
		}

		public void showTooltip(boolean show, String text, java.awt.Point position) {
			tooltipAnnotationAttributes.setVisible(show);
			tooltip.setText(text);
			if (position != null) {
				tooltip.setScreenPoint(position);
			}
			setAttributes(tooltipAnnotationAttributes);
		}

		private java.awt.Point offset = new java.awt.Point(5, 5);

		private int getOffsetX() {
			return this.offset != null ? this.offset.x : 0;
		}

		protected int getOffsetY() {
			return this.offset != null ? this.offset.y : 0;
		}

		@Override
		protected void doRenderNow(DrawContext dc) {
			if (dc.getPickPoint() == null)
				return;
			this.getAttributes().setDrawOffset(new java.awt.Point(this.getBounds(dc).width / 2 + this.getOffsetX(), this.getOffsetY()));
			this.setScreenPoint(this.adjustDrawPointToViewport(dc.getPickPoint(), this.getBounds(dc), dc.getView().getViewport()));
			super.doRenderNow(dc);
		}

		protected java.awt.Point adjustDrawPointToViewport(java.awt.Point point, Rectangle bounds, Rectangle viewport) {
			int x = point.x;
			int y = (int) viewport.getHeight() - point.y - 1;

			if (x + this.getOffsetX() + bounds.getWidth() > viewport.getWidth())
				x = (int) (viewport.getWidth() - bounds.getWidth()) - 1 - this.getOffsetX();
			else if (x < 0)
				x = 0;

			if (y + this.getOffsetY() + bounds.getHeight() > viewport.getHeight())
				y = (int) (viewport.getHeight() - bounds.getHeight()) - 1 - this.getOffsetY();
			else if (y < 0)
				y = bounds.height;

			return new java.awt.Point(x, y);
		}
	}
}
