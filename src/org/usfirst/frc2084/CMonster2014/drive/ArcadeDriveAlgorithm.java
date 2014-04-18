/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive;

import edu.wpi.first.wpilibj.GenericHID;

/**
 *
 * @author ben
 */
public class ArcadeDriveAlgorithm extends DriveAlgorithm {

    public ArcadeDriveAlgorithm(DriveController controller) {
        super(controller);
    }

    /**
     * Arcade drive implements single stick driving. Given a single Joystick,
     * the class assumes the Y axis for the move value and the X axis for the
     * rotate value. (Should add more information here regarding the way that
     * arcade drive works.)
     *
     * @param stick The joystick to use for Arcade single-stick driving. The
     * Y-axis will be selected for forwards/backwards and the X-axis will be
     * selected for rotation rate.
     * @param squaredInputs If true, the sensitivity will be decreased for small
     * values
     */
    public void arcadeDrive(GenericHID stick, boolean squaredInputs) {
        // simply call the full-featured arcadeDrive with the appropriate values
        arcadeDrive(stick.getY(), stick.getX(), squaredInputs);
    }

    /**
     * Arcade drive implements single stick driving. Given a single Joystick,
     * the class assumes the Y axis for the move value and the X axis for the
     * rotate value. (Should add more information here regarding the way that
     * arcade drive works.)
     *
     * @param stick The joystick to use for Arcade single-stick driving. The
     * Y-axis will be selected for forwards/backwards and the X-axis will be
     * selected for rotation rate.
     */
    public void arcadeDrive(GenericHID stick) {
        this.arcadeDrive(stick, true);
    }

    /**
     * Arcade drive implements single stick driving. Given two joystick
     * instances and two axis, compute the values to send to either two or four
     * motors.
     *
     * @param moveStick The Joystick object that represents the forward/backward
     * direction
     * @param moveAxis The axis on the moveStick object to use for
     * forwards/backwards (typically Y_AXIS)
     * @param rotateStick The Joystick object that represents the rotation value
     * @param rotateAxis The axis on the rotation object to use for the rotate
     * right/left (typically X_AXIS)
     * @param squaredInputs Setting this parameter to true decreases the
     * sensitivity at lower speeds
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis,
            boolean squaredInputs) {
        double moveValue = moveStick.getRawAxis(moveAxis);
        double rotateValue = rotateStick.getRawAxis(rotateAxis);

        arcadeDrive(moveValue, rotateValue, squaredInputs);
    }

    /**
     * Arcade drive implements single stick driving. Given two joystick
     * instances and two axis, compute the values to send to either two or four
     * motors.
     *
     * @param moveStick The Joystick object that represents the forward/backward
     * direction
     * @param moveAxis The axis on the moveStick object to use for
     * forwards/backwards (typically Y_AXIS)
     * @param rotateStick The Joystick object that represents the rotation value
     * @param rotateAxis The axis on the rotation object to use for the rotate
     * right/left (typically X_AXIS)
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis) {
        this.arcadeDrive(moveStick, moveAxis, rotateStick, rotateAxis, true);
    }

    /**
     * Arcade drive implements single stick driving. This function lets you
     * directly provide joystick values from any source.
     *
     * @param moveValue The value to use for forwards/backwards
     * @param rotateValue The value to use for the rotate right/left
     * @param squaredInputs If set, decreases the sensitivity at low speeds
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {

        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = DriveUtils.limit(moveValue);
        rotateValue = DriveUtils.limit(rotateValue);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

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

        controller.drive(leftMotorSpeed, rightMotorSpeed);
    }

    /**
     * Arcade drive implements single stick driving. This function lets you
     * directly provide joystick values from any source.
     *
     * @param moveValue The value to use for fowards/backwards
     * @param rotateValue The value to use for the rotate right/left
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
        this.arcadeDrive(moveValue, rotateValue, true);
    }
}
