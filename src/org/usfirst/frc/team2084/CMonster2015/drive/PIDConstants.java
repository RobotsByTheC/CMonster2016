/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

/**
 * @author ben
 */
public class PIDConstants {

	public final double p;
	public final double i;
	public final double d;
	public final double f;

	/**
	 * 
	 */
	public PIDConstants(double p, double i, double d, double f) {
		this.p = p;
		this.i = i;
		this.d = d;
		this.f = f;
	}

	/**
	 * 
	 */
	public PIDConstants(double p, double i, double d) {
		this(p, i, d, 0);
	}
}
