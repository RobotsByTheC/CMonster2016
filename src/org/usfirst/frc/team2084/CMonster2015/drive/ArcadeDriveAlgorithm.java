/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Implements the control of a drive controller using with arcade style
 * controls. This allows the use of one HID (Human Interface Device) to control
 * the robot, which is typically a joystick or gamepad. The Y-axis is usually
 * configured to the control the move speed (forward or reverse) and the X-axis
 * is used for rotation.
 *
 * @see DriveAlgorithm
 *
 * @author Ben Wolsieffer
 */
public class ArcadeDriveAlgorithm extends DriveAlgorithm<DriveController<? extends WheelController<? extends SpeedController>>> {

    /**
     * Creates a new {@link ArcadeDriveAlgorithm} that controls the specified
     * {@link DriveController}. This algorithm can use any type of
     * {@link DriveController}, since it only needs independent control over
     * each side of the robot.
     *
     * @param driveController the {@link DriveController} to control
     */
    public ArcadeDriveAlgorithm(
            DriveController<? extends WheelController<? extends SpeedController>> driveController) {
        super(driveController);
    }

    /**
     * Uses the X and Y axis of an HID, such as a joystick to drive.
     *
     * @param stick the joystick to use
     */
    public void arcadeDrive(GenericHID stick) {
        arcadeDrive(stick.getY(), stick.getX());
    }

    /**
     * Uses two separate HIDs to control move and rotation speeds. This method
     * also allows the selection of which axis to use on each stick.
     *
     * @param moveStick the HID that controls the move speed
     * @param moveAxis the axis on the {@code moveStick} to use for the move
     *            speed (typically Y_AXIS)
     * @param rotateStick The HID that controls the rotation speed
     * @param rotateAxis The axis on the {@code rotateStick} to use for the
     *            rotate speed (typically X_AXIS)
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis, GenericHID rotateStick,
            final int rotateAxis) {
        arcadeDrive(moveStick.getRawAxis(moveAxis), rotateStick.getRawAxis(rotateAxis));
    }

    /**
     * Uses the specified values to control the move and rotation speed of the
     * robot.
     *
     * @param moveSpeed the move speed
     * @param rotateSpeed the rotation speed
     */
    public void arcadeDrive(double moveSpeed, double rotateSpeed) {

        double leftMotorSpeed;
        double rightMotorSpeed;

        rotateSpeed *= -1;

        // Limit the inputs to the range of -1.0 to 1.0.
        moveSpeed = DriveUtils.limit(moveSpeed);
        rotateSpeed = DriveUtils.limit(rotateSpeed);

        // Do the calculations for arcade drive.
        if (moveSpeed > 0.0) {
            if (rotateSpeed > 0.0) {
                leftMotorSpeed = moveSpeed - rotateSpeed;
                rightMotorSpeed = Math.max(moveSpeed, rotateSpeed);
            } else {
                leftMotorSpeed = Math.max(moveSpeed, -rotateSpeed);
                rightMotorSpeed = moveSpeed + rotateSpeed;
            }
        } else {
            if (rotateSpeed > 0.0) {
                leftMotorSpeed = -Math.max(-moveSpeed, rotateSpeed);
                rightMotorSpeed = moveSpeed + rotateSpeed;
            } else {
                leftMotorSpeed = moveSpeed - rotateSpeed;
                rightMotorSpeed = -Math.max(-moveSpeed, -rotateSpeed);
            }
        }

        // Drive the left and right sides of the robot at the specified speeds.
        controller.drive(leftMotorSpeed, rightMotorSpeed);
    }
}
