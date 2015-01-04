/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.drive.processors;


/**
 * Applies a deadband to a value and scales it to be between 0.0 and 1.0 (or
 * -1.0, if the input was negative). <br>
 * <br>
 * For example, if {@code deadband = 0.1}:
 * <table border="1" summary="">
 * <tr>
 * <td>{@code process(0.0) = 0.0}</td>
 * </tr>
 * <tr>
 * <td>{@code process(0.1) = 0.0}</td>
 * </tr>
 * <tr>
 * <td>{@code process(0.5) = 0.444444}</td>
 * </tr>
 * <tr>
 * <td>{@code process(1.0) = 1.0}</td>
 * </tr>
 * </table>
 *
 * @author Ben Wolsieffer
 */
public class RescalingDeadband implements TimelessProcessor {

	private final double deadband;
	private final double absMax;

	public RescalingDeadband(double deadband) {
		this(deadband, 1);
	}

	public RescalingDeadband(double deadband, double absMax) {
		if (deadband < 0 || deadband > absMax) {
			throw new IllegalArgumentException("Deadband must be between 0.0 and " + absMax);
		}
		this.deadband = deadband;
		this.absMax = absMax;
	}

	@Override
	public double process(double value) {
		if (Math.abs(value) < deadband) {
			value = 0;
		} else {
			value += value < 0 ? deadband : -deadband;
			value /= absMax - deadband;
		}
		return value;
	}

}
