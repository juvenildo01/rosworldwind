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
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Hashtable;
import java.util.Vector;

import javax.swing.JButton;
import javax.swing.JPanel;
import javax.swing.JScrollPane;

import org.ros.worldwind.Adapter;
import org.ros.worldwind.AdapterProperties;
import org.ros.worldwind.ui.elements.AbstractElement;
import org.ros.worldwind.ui.elements.GlobalOptionsElement;
import org.ros.worldwind.ui.elements.MarkerElement;

public class Displays extends TopLevelPanel {
	private static final long serialVersionUID = 7483034187504602873L;

	private static final String MARKERS_KEY = "markers";

	public static final String LAYOUT_BUTTON = "height 18!";

	private JPanel elements;
	private Vector<Integer> elementsVector;
	private Hashtable<Integer, ElementPanel> elementPanelHashtable;
	private DeleteActionListener deleteActionListener;
	private Vector<AbstractElement> abstractElements;

	public Displays(Adapter adapter) {
		super(adapter);

		deleteActionListener = new DeleteActionListener();

		abstractElements = new Vector<AbstractElement>();

		setTitle("Displays");

		elements = generatePanel();
		AbstractElement abstractElement = new GlobalOptionsElement(getAdapter(), "globalOptions");
		elements.add(new ElementPanel(getAdapter(), "global options", abstractElement, "globalOptions", new Color(255, 255, 255), new Color(140, 140,
				140), new Color(0, 0, 0), new Color(255, 255, 255), null, ""));
		abstractElements.add(abstractElement);
		loadElements();
		JScrollPane scrollPane = new JScrollPane(elements, JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
		scrollPane.getVerticalScrollBar().setUnitIncrement(10);
		add(scrollPane);

		JPanel control = generatePanel();
		JButton addButton = new JButton("add");
		addButton.addActionListener(new AddActionListener());
		control.add(addButton);
		add(addButton);
	}

	public void updateOrigin() {
		for (AbstractElement abstractElement : abstractElements)
			abstractElement.updateOrigin();
	}

	private void loadElements() {
		elementsVector = new Vector<Integer>();
		elementPanelHashtable = new Hashtable<Integer, ElementPanel>();
		String markers = AdapterProperties.getInstance().getProperty(MARKERS_KEY);
		if (markers != null) {
			for (String marker : markers.split(",")) {
				int key = Integer.parseInt(marker);
				elementsVector.add(key);
				elementPanelHashtable.put(key, addElement("marker" + marker));
			}
		}
	}

	private ElementPanel addElement(String elementNumber) {
		MarkerElement marker = new MarkerElement(getAdapter(), elementNumber);
		Vector<ActionListener> deleteActionListeners = new Vector<ActionListener>();
		deleteActionListeners.add(deleteActionListener);
		deleteActionListeners.add(marker.getDeleteActionListener());
		ElementPanel elementPanel = new ElementPanel(getAdapter(), elementNumber, marker, elementNumber, new Color(255, 255, 255), new Color(180,
				180, 180), new Color(255, 100, 0), new Color(40, 40, 40), deleteActionListeners, "delete marker");
		abstractElements.add(marker);
		elements.add(elementPanel);
		return elementPanel;
	}

	private void store(String key, Vector<Integer> vector) {
		String value = "";
		for (Integer i : vector) {
			value += i + ",";
		}
		if (value.length() > 0)
			value = value.substring(0, value.lastIndexOf(","));
		AdapterProperties.getInstance().setProperty(key, value);
	}

	private class AddActionListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			int freeInteger = 0;
			for (Integer i : elementsVector) {
				if (i == freeInteger)
					freeInteger = i + 1;
				else {
					break;
				}
			}
			elementsVector.add(freeInteger, freeInteger);
			store(MARKERS_KEY, elementsVector);
			ElementPanel elementPanel = addElement("marker" + freeInteger);
			elementPanel.setVisibility(true);
			revalidate();
			repaint();
			elementPanelHashtable.put(freeInteger, elementPanel);
		}
	}

	private class DeleteActionListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			int index = Integer.parseInt(e.getActionCommand().substring(6, e.getActionCommand().length()));
			for (Integer i : elementsVector) {
				if (i == index) {
					elementsVector.remove(i);
					store(MARKERS_KEY, elementsVector);
					break;
				}
			}
			elements.remove(elementPanelHashtable.get(index));
			revalidate();
			repaint();
			elementPanelHashtable.remove(index);
		}
	}
}
