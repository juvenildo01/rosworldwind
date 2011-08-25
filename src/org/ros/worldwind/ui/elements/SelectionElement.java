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

import java.awt.event.ActionListener;
import java.text.DecimalFormat;
import java.text.NumberFormat;

import javax.swing.JButton;
import javax.swing.JLabel;

import org.ros.message.visualization_msgs.Marker;
import org.ros.worldwind.Adapter;
import org.ros.worldwind.ui.Displays;

public class SelectionElement extends AbstractElement {
	private static final long serialVersionUID = 4347534900533769635L;

	private NumberFormat numberFormat;

	private JLabel header_seq;
	private JLabel header_stamp_secs;
	private JLabel header_stamp_nsecs;
	private JLabel header_frameID;
	private JLabel namespace;
	private JLabel ID;
	private JLabel type;
	private JLabel action;
	private JLabel point_x;
	private JLabel point_y;
	private JLabel point_z;
	private JLabel orientation_x;
	private JLabel orientation_y;
	private JLabel orientation_z;
	private JLabel orientation_w;
	private JLabel scale_x;
	private JLabel scale_y;
	private JLabel scale_z;
	private JLabel color_r;
	private JLabel color_g;
	private JLabel color_b;
	private JLabel color_a;
	private JLabel frame_locked;

	public SelectionElement(Adapter adapter, String persistenceID, ActionListener deselectActionListener) {
		super(adapter, persistenceID);
		numberFormat = new DecimalFormat("#.################");

		header_seq = new JLabel();
		header_stamp_secs = new JLabel();
		header_stamp_nsecs = new JLabel();
		header_frameID = new JLabel();
		addRow(new JLabel("sequenz"), header_seq);
		addRow(new JLabel("secs"), header_stamp_secs);
		addRow(new JLabel("nsecs"), header_stamp_nsecs);
		addRow(new JLabel("frame ID"), header_frameID);

		namespace = new JLabel();
		ID = new JLabel();
		type = new JLabel();
		action = new JLabel();
		addRow(new JLabel("namespace"), namespace);
		addRow(new JLabel("ID"), ID);
		addRow(new JLabel("type"), type);
		addRow(new JLabel("action"), action);

		point_x = new JLabel();
		point_y = new JLabel();
		point_z = new JLabel();
		addRow(new JLabel("point x"), point_x);
		addRow(new JLabel("point y"), point_y);
		addRow(new JLabel("point z"), point_z);

		orientation_x = new JLabel();
		orientation_y = new JLabel();
		orientation_z = new JLabel();
		orientation_w = new JLabel();
		addRow(new JLabel("rotation x"), orientation_x);
		addRow(new JLabel("rotation y"), orientation_y);
		addRow(new JLabel("rotation z"), orientation_z);
		addRow(new JLabel("rotation w"), orientation_w);

		scale_x = new JLabel();
		scale_y = new JLabel();
		scale_z = new JLabel();
		addRow(new JLabel("scale x"), scale_x);
		addRow(new JLabel("scale y"), scale_y);
		addRow(new JLabel("scale z"), scale_z);

		color_r = new JLabel();
		color_g = new JLabel();
		color_b = new JLabel();
		color_a = new JLabel();
		addRow(new JLabel("color r"), color_r);
		addRow(new JLabel("color g"), color_g);
		addRow(new JLabel("color b"), color_b);
		addRow(new JLabel("color a"), color_a);

		frame_locked = new JLabel();
		addRow(new JLabel("frame lock"), frame_locked);

		JButton deselectButton = new JButton("deselect");
		deselectButton.addActionListener(deselectActionListener);
		deselectButton.setActionCommand(MarkerElement.ACTION_COMMAND_DESELECT);
		addRow(deselectButton, Displays.LAYOUT_BUTTON);
	}

	public void setSelection(Marker marker) {
		if (marker != null) {
			header_seq.setText(marker.header.seq + "");
			header_stamp_secs.setText(marker.header.stamp.secs + "");
			header_stamp_nsecs.setText(marker.header.stamp.nsecs + "");
			header_frameID.setText(marker.header.frame_id + "");
			namespace.setText(marker.ns + "");
			ID.setText(marker.id + "");
			type.setText(marker.type + "");
			action.setText(marker.action + "");
			point_x.setText(numberFormat.format(marker.pose.position.x) + "");
			point_y.setText(numberFormat.format(marker.pose.position.y) + "");
			point_z.setText(numberFormat.format(marker.pose.position.z) + "");
			orientation_x.setText(numberFormat.format(marker.pose.orientation.x) + "");
			orientation_y.setText(numberFormat.format(marker.pose.orientation.y) + "");
			orientation_z.setText(numberFormat.format(marker.pose.orientation.z) + "");
			orientation_w.setText(numberFormat.format(marker.pose.orientation.w) + "");
			scale_x.setText(numberFormat.format(marker.scale.x) + "");
			scale_y.setText(numberFormat.format(marker.scale.y) + "");
			scale_z.setText(numberFormat.format(marker.scale.z) + "");
			color_r.setText(marker.color.r + "");
			color_g.setText(marker.color.g + "");
			color_b.setText(marker.color.b + "");
			color_a.setText(marker.color.a + "");
			frame_locked.setText(marker.frame_locked + "");
		}
	}

	@Override
	public void updateOrigin() {
	}

}
