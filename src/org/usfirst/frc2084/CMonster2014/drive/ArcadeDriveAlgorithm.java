/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * Implements the control of a drive controller using with arcade style
 * controls. This allows the use of one HID (Human Interface Device) to control
 * the robot, which is typically a joystick or gamepad. The Y-axis is usually
 * configured to the control the move speed (forward or reverse) and the X-axis
 * is used for rotation. The algorithm also has the ability to square the inputs
 * to decrease the sensitivity at low speeds.
 *
 * @see DriveAlgorithm
 *
 * @author Ben Wolsieffer
 */
public class ArcadeDriveAlgorithm extends DriveAlgorithm {

    /**
     * Creates a new {@link ArcadeDriveAlgorithm} that controls the specified
     * {@link DriveController}. This algorithm can use any type of
     * {@link DriveController}, since it only needs independent control over
     * each side of the robot.
     *
     * @param controller
     */
    public ArcadeDriveAlgorithm(DriveController controller) {
        super(controller);
    }

    /**
     * Uses the X and Y axis of an HID, such as a joystick to drive. This method
     * also provides the option for squaring the axis values.
     *
     * @param stick the joystick to use
     * @param squaredInputs if true, the axes are squared to decrease the
     * sensitivity at low speeds
     */
    public void arcadeDrive(GenericHID stick, boolean squaredInputs) {
        arcadeDrive(stick.getY(), stick.getX(), squaredInputs);
    }

    /**
     * Uses the X and Y axis of an HID, such as a joystick to drive. This method
     * squares the axis values.
     *
     * @param stick the joystick to use
     */
    public void arcadeDrive(GenericHID stick) {
        this.arcadeDrive(stick, true);
    }

    /**
     * Uses two separate HIDs to control move and rotation speeds. This method
     * also allows the selection of which axis to use on each stick and whether
     * to square the axis values.
     *
     * @param moveStick the HID that controls the move speed
     * @param moveAxis the axis on the {@code moveStick} to use for the move
     * speed (typically Y_AXIS)
     * @param rotateStick The HID that controls the rotation speed
     * @param rotateAxis The axis on the {@code rotateStick} to use for the
     * rotate speed (typically X_AXIS)
     * @param squaredInputs whether to square the axes
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis,
            boolean squaredInputs) {
        arcadeDrive(moveStick.getRawAxis(moveAxis), rotateStick.getRawAxis(rotateAxis), squaredInputs);
    }

    /**
     * Uses two separate HIDs to control move and rotation. This method also
     * allows the selection of which axis to use on each stick. This method
     * squares the axis values.
     *
     * @param moveStick the HID that controls the move value
     * @param moveAxis the axis on the {@code moveStick} to use for the move
     * value (typically Y_AXIS)
     * @param rotateStick The HID that controls the rotation value
     * @param rotateAxis The axis on the {@code rotateStick} to use for the
     * rotate value (typically X_AXIS)
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis) {
        this.arcadeDrive(moveStick, moveAxis, rotateStick, rotateAxis, true);
    }

    /**
     * Uses the specified values to control the move and rotation speed of the
     * robot and whether to square the inputs.
     *
     * @param moveValue the move speed
     * @param rotateValue the rotation speed
     * @param squaredInputs if true, the inputs are squared to decrease the
     * sensitivity at low speeds
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {

        double leftMotorSpeed;
        double rightMotorSpeed;

        // Limit the inputs to the range of -1.0 to 1.0.
        moveValue = DriveUtils.limit(moveValue);
        rotateValue = DriveUtils.limit(rotateValue);

        if (squaredInputs) {
            // Square the inputs (while preserving the sign) to increase fine 
            // control while still permitting full power.
            moveValue *= moveValue * (moveValue >= 0 ? 1 : -1);
            rotateValue *= rotateValue * (rotateValue >= 0 ? 1 : -1);
        }

        // Do the calculations for arcade drive.
        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        // Drive the left and right sides of the robot at the specified speeds.
        controller.drive(leftMotorSpeed, rightMotorSpeed);
    }

    /**
     * Uses the specified values to control the move and rotation speed of the
     * robot and whether to square the inputs. This method squares the provided
     * inputs.
     *
     * @param moveValue the move speed
     * @param rotateValue the rotation speed
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
        this.arcadeDrive(moveValue, rotateValue, true);
    }
}
