/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;

/**
 * Controls a set of wheels. This class manages the watchdog and provides the
 * definition for a simple skid steer {@link #drive(double, double)} method that
 * all DriveControllers must implement.
 *
 * @see DriveAlgorithm
 * @see FourWheelDriveController
 * @see TwoWheelDriveController
 *
 * @author Ben Wolsieffer
 */
public abstract class DriveController<W extends WheelController<?>> implements MotorSafety {

    /**
     * Manages the watchdog timer for this {@link DriveController}. The watchdog
     * timer automatically disables the motors if they are not being updated.
     */
    protected MotorSafetyHelper safetyHelper;

    public DriveController() {
        safetyHelper = new MotorSafetyHelper(this);
        safetyHelper.setSafetyEnabled(true);
    }

    /**
     * Set the left and right speeds of the robot using the specified values.
     * This must be implemented in subclasses.
     *
     * @param leftSpeed the speed of the left side of the robot
     * @param rightSpeed the speed of the right side of the robot
     */
    public abstract void drive(double leftSpeed, double rightSpeed);

    /**
     * Sets the expiration time for the watchdog.
     *
     * @see MotorSafetyHelper#setExpiration(double)
     *
     * @param expirationTime the expiration time in seconds
     */
    @Override
    public final void setExpiration(double expirationTime) {
        safetyHelper.setExpiration(expirationTime);
    }

    /**
     * Gets the expiration time for the watchdog.
     *
     * @see MotorSafetyHelper#getExpiration()
     *
     * @return the expiration time in second
     */
    @Override
    public final double getExpiration() {
        return safetyHelper.getExpiration();
    }

    /**
     * Determine if the motor is still operating or has timed out.
     *
     * @see MotorSafetyHelper#isAlive()
     *
     * @return a true value if the motor is still operating normally and hasn't
     *         timed out
     */
    @Override
    public final boolean isAlive() {
        return safetyHelper.isAlive();
    }

    /**
     * Proxy that calls the real stop method when the watchdog times out.
     */
    @Override
    public final void stopMotor() {
        stop();
    }

    /**
     * Stops the robot and is called if the watchdog times out. Must be
     * implemented by subclasses.
     */
    public abstract void stop();

    /**
     * Enables or disables the watchdog for this {@link DriveController}.
     *
     * @param enabled true if motor safety is enforced for this object
     */
    @Override
    public final void setSafetyEnabled(boolean enabled) {
        safetyHelper.setSafetyEnabled(enabled);
    }

    /**
     * Returns if the watchdog is enabled for this {@link DriveController}.
     *
     * @return true if motor safety is enforced for this object
     */
    @Override
    public final boolean isSafetyEnabled() {
        return safetyHelper.isSafetyEnabled();
    }

    /**
     * Gets the description of this motor safety instance. By default it is the
     * class name.
     *
     * @return the class name
     */
    @Override
    public String getDescription() {
        return getClass().getName();
    }
}
