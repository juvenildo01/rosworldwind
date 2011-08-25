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

import javax.swing.JPanel;

import net.miginfocom.swing.MigLayout;

import org.ros.worldwind.Adapter;

public class TopLevelPanel extends JPanel {
	private static final long serialVersionUID = -1016309288100135724L;

	private Adapter adapter;

	protected static final String LAYOUT_CONSTRAINTS = "hidemode 3, gap 0, novisualpadding, ins 0, wrap 1";
	protected static final String COL_CONSTRAINTS = "[fill, grow]";
	protected static final String ROW_CONSTRAINTS = "[]";

	public TopLevelPanel(Adapter adapter) {
		setAdapter(adapter);

		setLayout(new MigLayout("hidemode 3, gap 0, novisualpadding, ins 2, wrap 1", "[fill, grow]", "[][fill, grow][]"));
	}

	public Adapter getAdapter() {
		return adapter;
	}

	public void setAdapter(Adapter adapter) {
		this.adapter = adapter;
	}

	protected void setTitle(String title) {
		add(UITools.generateTitleLabel(title, new JPanel(new MigLayout("hidemode 3, novisualpadding, ins 2", "[fill, grow]", "[fill, grow]")),
				new Color(0.2f, 0.2f, 0.2f, 1f), new Color(1f, 1f, 1f, 1f)));
	}

	protected JPanel generatePanel() {
		return new JPanel(new MigLayout(LAYOUT_CONSTRAINTS, COL_CONSTRAINTS, ROW_CONSTRAINTS));
	}
}
