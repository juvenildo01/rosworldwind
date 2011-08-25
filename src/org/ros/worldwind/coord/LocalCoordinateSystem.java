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
package org.ros.worldwind.coord;

import gov.nasa.worldwind.geom.Angle;
import gov.nasa.worldwind.geom.Matrix;
import gov.nasa.worldwind.geom.Position;
import gov.nasa.worldwind.geom.Vec4;
import gov.nasa.worldwind.globes.Globe;

public class LocalCoordinateSystem {

	private Angle roll;
	private Angle tilt;
	private Angle heading;
	private Globe globe;
	private Vec4 originVector;
	private Matrix defaultTransformation;
	private Position originPosition;

	public LocalCoordinateSystem(Globe globe) {
		this.globe = globe;
	}

	public void setLocalOrigin(Position position) {
		setLocalOrigin(position, Angle.fromDegrees(0), Angle.fromDegrees(0), Angle.fromDegrees(0));
	}

	public void setLocalOrigin(Position position, Angle roll, Angle tilt, Angle heading) {
		this.roll = roll;
		this.tilt = tilt;
		this.heading = heading;
		this.originVector = globe.computePointFromPosition(position);
		this.originPosition = position;
		this.defaultTransformation = globe.computeSurfaceOrientationAtPosition(position);
	}

	public Position getOriginPosition() {
		return originPosition;
	}

	public Vec4 getModelCoordinateVectorToOrigin() {
		return originVector;
	}

	public Position transform(Vec4 point) {

		return globe.computePositionFromPoint(transform2Vec(point));
	}

	public Vec4 transform2Vec(Vec4 point) {
		Matrix matrix = defaultTransformation;

		if (tilt != null)
			matrix = matrix.multiply(Matrix.fromRotationX(Angle.POS360.subtract(this.roll)));
		if (roll != null)
			matrix = matrix.multiply(Matrix.fromRotationY(Angle.POS360.subtract(this.tilt)));
		if (heading != null)
			matrix = matrix.multiply(Matrix.fromRotationZ(Angle.POS360.subtract(this.heading)));

		return point.transformBy4(matrix);
	}

	public Matrix transform(Matrix rotation) {
		return defaultTransformation.multiply(rotation);
	}

	public Matrix getRotation() {
		return Matrix.fromRotationXYZ(roll, tilt, heading);
	}

}
