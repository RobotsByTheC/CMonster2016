/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive;

/**
 * Controls a robot that has two wheels, or (in most cases) two sides that each
 * act as a unit, making the control the same as if the robot only had two
 * wheels.
 *
 * @author Ben Wolsieffer
 */
public class TwoWheelDriveController extends DriveController {

    protected WheelController leftWheel;
    protected WheelController rightWheel;

    /**
     * Creates a new {@link TwoWheelDriveController} that drives the specified
     * {@link WheelController}s.
     *
     * @param leftWheel the left {@link WheelController}
     * @param rightWheel the right {@link WheelController}
     */
    public TwoWheelDriveController(WheelController leftWheel, WheelController rightWheel) {
        this.leftWheel = leftWheel;
        this.rightWheel = rightWheel;
    }

    /**
     * Drives the left and right {@link WheelController}s at the specified
     * speeds.
     *
     * @param leftSpeed the left wheel speed
     * @param rightSpeed the right wheel speed
     */
    public void drive(double leftSpeed, double rightSpeed) {
        leftWheel.set(leftSpeed);
        rightWheel.set(rightSpeed);

        // Make sure to feed the watchdog!
        safetyHelper.feed();
    }

    /**
     * Stops the robot.
     */
    public void stopMotor() {
        leftWheel.set(0);
        rightWheel.set(0);
    }

    public String getDescription() {
        return "Two Wheel Drive Controller";
    }
}
