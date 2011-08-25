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
package org.ros.worldwind.ui;

import java.awt.Color;
import java.awt.event.ActionListener;
import java.util.Hashtable;

import javax.swing.JPanel;
import javax.swing.JScrollPane;

import org.ros.message.visualization_msgs.Marker;
import org.ros.worldwind.Adapter;
import org.ros.worldwind.ui.elements.SelectionElement;

public class Selection extends TopLevelPanel {
	private static final long serialVersionUID = 4093426903466567562L;

	private JPanel selection;
	private Hashtable<String, ElementPanel> selectionElementPanels;
	private Hashtable<String, SelectionElement> selectionElements;

	public Selection(Adapter adapter) {
		super(adapter);
		setTitle("Selection");

		selectionElementPanels = new Hashtable<String, ElementPanel>();
		selectionElements = new Hashtable<String, SelectionElement>();

		selection = generatePanel();
		JScrollPane scrollPane = new JScrollPane(selection, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
		scrollPane.getVerticalScrollBar().setUnitIncrement(10);
		add(scrollPane);
	}

	public void addSelection(String ID, Marker marker, ActionListener deselectActionListener) {
		if (!selectionElements.containsKey(ID)) {
			SelectionElement selectionElement = new SelectionElement(getAdapter(), "", deselectActionListener);
			ElementPanel elementPanel = new ElementPanel(getAdapter(), ID, selectionElement, "", new Color(255, 255, 255), new Color(180, 180, 180),
					new Color(40, 40, 40), new Color(240, 240, 240), null, "");
			elementPanel.setVisibility(true);
			selection.add(elementPanel);
			selectionElementPanels.put(ID, elementPanel);
			selectionElements.put(ID, selectionElement);
		}
		updateSelection(ID, marker);
	}

	public void updateSelection(String ID, Marker marker) {
		if (selectionElements.containsKey(ID))
			selectionElements.get(ID).setSelection(marker);
	}

	public void removeSelection(String ID) {
		if (selectionElements.containsKey(ID)) {
			selection.remove(selectionElementPanels.get(ID));
			selectionElementPanels.remove(ID);
			selectionElements.remove(ID);
		}
		selection.revalidate();
		selection.repaint();
	}

}
