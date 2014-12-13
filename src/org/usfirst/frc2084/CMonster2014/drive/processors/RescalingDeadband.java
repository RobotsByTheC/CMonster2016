/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive.processors;

import org.usfirst.frc2084.CMonster2014.drive.DriveUtils;

/**
 * Applies a deadband to a value and scales it to be between 0.0 and 1.0 (or
 * -1.0, if the input was negative).
 * <br/>
 * <br/>
 * For example, if {@code deadband = 0.1}:
 * <table border="1">
 * <tr><td>{@code process(0.0) = 0.0}</td></tr>
 * <tr><td>{@code process(0.1) = 0.0}</td></tr>
 * <tr><td>{@code process(0.5) = 0.444444}</td></tr>
 * <tr><td>{@code process(1.0) = 1.0}</td></tr>
 * </table>
 *
 * @author Ben Wolsieffer
 */
public class RescalingDeadband implements ValueProcessor {

    private final double deadband;

    public RescalingDeadband(double deadband) {
        if (!DriveUtils.isValid(deadband)) {
            throw new IllegalArgumentException("Deadband must be between -1.0 and 1.0");
        }
        this.deadband = deadband;
    }

    public double process(double value) {
        if (Math.abs(value) < deadband) {
            value = 0;
        } else {
            value += value < 0 ? deadband : -deadband;
            value /= 1 - deadband;
        }
        return value;
    }

}
