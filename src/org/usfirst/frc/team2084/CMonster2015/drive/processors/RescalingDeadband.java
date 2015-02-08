/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive.processors;

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
 * <td>{@code process(0.5) = 0.4444...}</td>
 * </tr>
 * <tr>
 * <td>{@code process(1.0) = 1.0}</td>
 * </tr>
 * </table>
 *
 * @author Ben Wolsieffer
 */
public class RescalingDeadband implements Processor {

    /**
     * The deadband value.
     */
    private final double deadband;

    /**
     * Creates a {@link RescalingDeadband} with the specified deadband value.
     * 
     * @param deadband the deadband value
     */
    public RescalingDeadband(double deadband) {
        if (deadband < 0) {
            throw new IllegalArgumentException("Deadband must be greater than 0");
        }
        this.deadband = deadband;
    }

    /**
     * Applies a deadband to the input value.
     * 
     * @param value the input value
     * @return the value with deadband applied
     */
    @Override
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
