/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A {@link DriveAlgorithm} that uses a {@link FourWheelDriveController} (which
 * controls four mecanum wheels) to drive using a gyro to maintain orientation
 * and drive relative to the field. Mecanum wheels have rollers set at a 45
 * degree angle from the wheel's direction of rotation. This allows them the
 * robot to move in any direction.
 *
 * @see FourWheelDriveController
 * @see DriveAlgorithm
 *
 * @author Ben Wolsieffer
 */
public class MecanumDriveAlgorithm extends DriveAlgorithm {

    /**
     * The {@link MecanumDriveAlgorithm} requires a
     * {@link FourWheelDriveController} so the normal
     * {@link DriveAlgorithm#controller} is hidden.
     */
    protected final FourWheelDriveController controller;
    /**
     * The {@link Gyro} that the {@link MecanumDriveAlgorithm} uses for
     * field-oriented driving and keeping the correct orientation.
     */
    protected final Gyro gyro;

    /**
     * The rotation speed below which the rotation PID controller is enabled.
     * When the rotation speed increases above this value the controller is
     * disabled to allow the robot to turn.
     */
    public final double ROTATION_DEADBAND = 0.05;
    /**
     * The proportional constant of the PID algorithm. This value is multiplied
     * by the error.
     */
    public static final double ROTATION_P = 0.01;
    /**
     * The integral constant of the PID algorithm. This value is multiplied by
     * the sum of the error over time. This is used to make the robot turn more
     * if it is taking a long time to reach its target.
     */
    public static final double ROTATION_I = 0.0;
    /**
     * The derivative constant of the PID algorithm. This value is multiplied by
     * the rate of change of the error.
     */
    public static final double ROTATION_D = 0.0;
    /**
     * The feed-forward constant of the PID algorithm. This value seems to be
     * multiplied by the setpoint, which doesn't make much sense. I don't really
     * know what it is supposed to do so I just leave it at 0.
     */
    public static final double ROTATION_F = 0.0;
    private double rotationSpeedPID = 0.0;
    private double gyroOffset = 0.0;

    /**
     * {@link http://www.chiefdelphi.com/media/papers/download/1829}
     */
    private final PIDController rotationPIDController;

    /**
     * Creates a new {@link MecanumDriveAlgorithm} that controls the specified
     * {@link FourWheelDriveController}.
     *
     * @param controller the {@link FourWheelDriveController} to control
     * @param gyro the {@link Gyro} to use for orientation correction and
     * field-oriented driving
     */
    public MecanumDriveAlgorithm(FourWheelDriveController controller, Gyro gyro) {
        super(controller);
        // Necessary because we hide the controller field inherited from
        // DriveAlgorithm (if this was >=Java 5 I would use generics).
        this.controller = controller;
        this.gyro = gyro;
        rotationPIDController = new PIDController(
                ROTATION_P,
                ROTATION_I,
                ROTATION_D,
                ROTATION_F,
                gyro,
                new PIDOutput() {
                    public void pidWrite(double output) {
                        rotationSpeedPID = output;
                    }
                }
        );
        SmartDashboard.putData("Mecanum Drive Controller", rotationPIDController);
    }

    /**
     * Moves the robot forward and sideways while rotating at the specified
     * speeds. This method uses the specified angle to implement field oriented
     * controls.
     *
     * @param x The forward speed (negative = backward, positive = forward)
     * @param y The sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     * clockwise, positive = counterclockwise)
     * @param gyroAngle the current angle reading from the gyro
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
        rotation = getRotationPID(rotation);
        mecanumDrive_Cartesian0(x, y, rotation, gyroAngle);
    }

    /**
     * Moves the robot forward and sideways while rotating at the specified
     * speeds. This moves the robot relative to the robot's current orientation.
     *
     * @param x The forward speed (negative = backward, positive = forward)
     * @param y The sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     * clockwise, positive = counterclockwise)
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation) {
        mecanumDrive_Cartesian(x, y, rotation, gyro.getAngle() - gyroOffset);
    }

    public void mecanumDrive_Orientation(double x, double y, double angle) {
        if (!rotationPIDController.isEnable() || rotationPIDController.getSetpoint() != angle) {
            rotationPIDController.setSetpoint(angle);
            rotationPIDController.enable();
        }

        mecanumDrive_Cartesian0(x, y, rotationSpeedPID, gyro.getAngle());
    }

    /**
     * Real implementation of cartesian mecanum driving. This method does not
     * include gyro orientation correction and it is only called from with this
     * class.
     *
     * @param x The forward speed (negative = backward, positive = forward)
     * @param y The sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (positive =
     * clockwise, negative = counterclockwise)
     * @param gyroAngle the current angle reading from the gyro
     */
    private void mecanumDrive_Cartesian0(double x, double y, double rotation, double gyroAngle) {
        // Compenstate for gyro angle.
        double rotated[] = DriveUtils.rotateVector(x, y, gyroAngle);
        x = rotated[0];
        y = rotated[1];

        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = x + y - rotation;
        wheelSpeeds[1] = -x + y + rotation;
        wheelSpeeds[2] = -x + y - rotation;
        wheelSpeeds[3] = x + y + rotation;

        DriveUtils.normalize(wheelSpeeds);

        controller.drive(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
    }

    /**
     * Drives the robot at the specified speed in the direction specified as an
     * angle in degrees while rotating. This does not take into account the gyro
     * correction yet because we do not use it.
     *
     * @param magnitude The speed that the robot should drive in a given
     * direction.
     * @param direction the direction the robot should drive in degrees,
     * independent of rotation
     * @param rotation The rate of rotation for the robot that is completely
     * independent of the magnitude or direction. [-1.0..1.0]
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation) {
        // Normalized for full power along the Cartesian axes.
        magnitude = DriveUtils.limit(magnitude) * Math.sqrt(2.0);
        // The rollers are at 45 degree angles.
        double dirInRad = (direction + 45.0) * Math.PI / 180.0;
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = (sinD * magnitude + rotation);
        wheelSpeeds[1] = (cosD * magnitude - rotation);
        wheelSpeeds[2] = (cosD * magnitude + rotation);
        wheelSpeeds[3] = (sinD * magnitude - rotation);

        DriveUtils.normalize(wheelSpeeds);

        controller.drive(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
    }

    /**
     * Drive based on the specified joystick using the x and y axes.
     *
     * @param stick The joystick to use
     */
    public void mecanumDrive_Cartesian(GenericHID stick) {
        mecanumDrive_Cartesian(stick.getX(), stick.getY());
    }

    /**
     * Moves the robot forward and sideways at the specified speeds.
     *
     * @param x The forward speed (negative = backward, positive = forward)
     * @param y The sideways (crab) speed (negative = left, positive = right)
     *
     */
    public void mecanumDrive_Cartesian(double x, double y) {
        mecanumDrive_Cartesian(x, y, 0);
    }

    /**
     * Moves the robot sideways at the specified speed.
     *
     * @param speed The speed and direction to crab (negative = left, positive =
     * right)
     */
    public void crab(double speed) {
        mecanumDrive_Cartesian(speed, 0);
    }

    /**
     * Gets the corrected rotation speed based on the gyro heading and the
     * expected rate of rotation. If the rotation rate is above a threshold, the
     * gyro correction is turned off.
     *
     * @param rotationSpeed
     * @return
     */
    private double getRotationPID(double rotationSpeed) {
        // If the controller is already enabled, check to see if it should be 
        // disabled  or kept running. Otherwise check to see if it needs to be 
        // enabled.
        if (rotationPIDController.isEnable()) {
            // If the rotation rate is greater than the deadband disable the PID
            // controller. Otherwise, return the latest value from the
            // controller.
            if (Math.abs(rotationSpeed) >= ROTATION_DEADBAND) {
                rotationPIDController.disable();
            } else {
                return rotationSpeedPID;
            }
        } else {
            // If the rotation rate is less than the deadband, turn on the PID
            // controller and set its setpoint to the current angle.
            if (Math.abs(rotationSpeed) < ROTATION_DEADBAND) {
                gyroOffset = gyro.getAngle();
                rotationPIDController.setSetpoint(gyroOffset);
                rotationPIDController.enable();
            }
        }
        // Unless told otherwise, return the rate that was passed in.
        return rotationSpeed;
    }

    /**
     * Resets the robot's gyro value to zero and resets the PID controller
     * error. This is usually called on command, or after the robot has been
     * disabled to get rid of drift.
     */
    public void resetGyro() {
        // Reset the gyro value to zero
        gyro.reset();
        // Reset the integral component to zero (which also disables the 
        // controller). This is very important because the integral value will
        // have gotten really big and will cause the robot to spin in circles
        // unless it is reset.
        rotationPIDController.reset();
        // Since the gyro value is now zero, the robot should also try to point 
        // in that direction.
        rotationPIDController.setSetpoint(0);
        // Renable the controller because it was disabled by calling reset().
        rotationPIDController.enable();
    }
}
