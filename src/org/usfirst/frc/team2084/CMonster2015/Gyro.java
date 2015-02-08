/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015;

import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
 * Represents any type of single axis gyro. Multi-axis gyros should expose
 * multiple instances of this class, each representing one axis. This is
 * designed to follow the same pattern as the {@link Accelerometer} class
 * provided by WPILib.
 * 
 * @author Ben Wolsieffer
 */
public interface Gyro {

    /**
     * Gets the angle of the gyro in radians.
     * 
     * @return the gyro angle
     */
    public double getAngle();

    /**
     * Gets the rate of rotation of the gyro in radians/second.
     * 
     * @return the rotation rate
     */
    public double getRate();

    /**
     * Resets the gyro angle to 0. This should be called to reset the effects of
     * significant drift.
     */
    public void reset();
}
