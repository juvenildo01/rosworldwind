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
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Vector;

import javax.swing.BorderFactory;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JPanel;

import net.miginfocom.swing.MigLayout;

import org.ros.worldwind.Adapter;
import org.ros.worldwind.AdapterProperties;

public class ElementPanel extends JPanel {
	private static final long serialVersionUID = 7846963120279299479L;

	public static final String VISIBILITY_KEY = "visibility";

	protected Adapter adapter;
	private String persistenceID;

	private JPanel control;
	private JPanel blindPanel;
	private JPanel content;
	private JButton deleteButton;

	private JButton collapseButton;
	private JButton expandButton;

	public ElementPanel(Adapter adapter, String title, JPanel content, String persistenceID, Color contentBackgroundColor, Color leftColor,
			Color titleBackgroundColor, Color titleFontColor, Vector<ActionListener> deleteActionListeners, String deleteButtonString) {
		this.adapter = adapter;
		this.content = content;
		this.persistenceID = persistenceID;

		setLayout(new MigLayout("hidemode 3, gap 0 0, novisualpadding, ins 0, wrap 2", "[][fill, grow]", "[][fill, grow][]"));

		control = new JPanel(new MigLayout("hidemode 3, novisualpadding, ins 4", "[]", "[]"));
		UITools.setPanelColor(control, leftColor);
		collapseButton = generateButton("gfx/controls/collapse.png", leftColor);
		expandButton = generateButton("gfx/controls/expand.png", leftColor);
		control.add(expandButton);
		add(control);

		add(UITools.generateTitleLabel(title, new JPanel(new MigLayout("hidemode 3, novisualpadding, ins 2", "[fill, grow]", "[fill, grow]")),
				titleBackgroundColor, titleFontColor));

		blindPanel = new JPanel(new MigLayout("hidemode 3, novisualpadding, ins 9", "[]", "[]"));
		UITools.setPanelColor(blindPanel, leftColor);
		String layoutConstraints = "";
		if (deleteActionListeners != null)
			layoutConstraints = "spany 2";
		add(blindPanel, layoutConstraints);

		UITools.setPanelColor(content, contentBackgroundColor);
		add(content);

		if (deleteActionListeners != null) {
			JPanel deletePanel = new JPanel(new MigLayout("hidemode 3, novisualpadding, ins 0 4 4 4", "[fill, grow]", "[]"));
			UITools.setPanelColor(deletePanel, new Color(1f, 1f, 1f, 0f));
			deleteButton = new JButton(deleteButtonString);
			for (ActionListener actionListener : deleteActionListeners)
				deleteButton.addActionListener(actionListener);
			deleteButton.addActionListener(new DeleteActionListener());
			deleteButton.setActionCommand(title);
			deletePanel.add(deleteButton, Displays.LAYOUT_BUTTON);
			UITools.setPanelColor(deletePanel, contentBackgroundColor);
			add(deletePanel);
		}
		setVisibility(Boolean.parseBoolean(AdapterProperties.getInstance().getProperty(persistenceID + "." + VISIBILITY_KEY)));
	}

	public void updateOrigin() {

	}

	private JButton generateButton(String gfxPath, Color color) {
		ImageIcon imageIcon = new ImageIcon(gfxPath);
		Dimension buttonDimension = new Dimension(imageIcon.getIconWidth(), imageIcon.getIconHeight());
		JButton button = new JButton(imageIcon);
		button.setIcon(imageIcon);
		button.setPressedIcon(imageIcon);
		button.setMinimumSize(buttonDimension);
		button.setPreferredSize(buttonDimension);
		button.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 0));
		button.setOpaque(false);
		button.setContentAreaFilled(false);
		button.setBackground(color);
		button.setFocusable(false);
		button.addActionListener(new CollapsedExpandActionListener());
		return button;
	}

	public void setVisibility(boolean visibility) {
		control.removeAll();
		control.add(visibility ? collapseButton : expandButton);
		content.setVisible(visibility);
		if (deleteButton != null)
			deleteButton.setVisible(visibility);
		blindPanel.setVisible(visibility);
		AdapterProperties.getInstance().setProperty(persistenceID + "." + VISIBILITY_KEY, visibility + "");
	}

	private class CollapsedExpandActionListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			setVisibility(!content.isVisible());
		}
	}

	private class DeleteActionListener implements ActionListener {
		@Override
		public void actionPerformed(ActionEvent e) {
			AdapterProperties.getInstance().removeProperty(persistenceID + "." + VISIBILITY_KEY);
		}
	}

}
