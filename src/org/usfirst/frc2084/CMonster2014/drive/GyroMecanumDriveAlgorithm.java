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
public class GyroMecanumDriveAlgorithm extends MecanumDriveAlgorithm {

    /**
     * The {@link Gyro} that the {@link GyroMecanumDriveAlgorithm} uses for
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
    public static final double ROTATION_P = 0.573;
    /**
     * The integral constant of the PID algorithm. This value is multiplied by
     * the sum of the error over time. This is used to make the robot turn more
     * if it is taking a long time to reach its target.
     */
    public static final double ROTATION_I = 0.0;
    /**
     * The derivative constant of the PID algorithm. This value is multiplied by
     * the rate of change of the error. This is used to make the robot respond
     * to sudden increases in error, but we are unlikely to use it.
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
     * Creates a new {@link GyroMecanumDriveAlgorithm} that controls the
     * specified {@link FourWheelDriveController}.
     *
     * @param controller the {@link FourWheelDriveController} to control
     * @param gyro the {@link Gyro} to use for orientation correction and
     * field-oriented driving
     */
    public GyroMecanumDriveAlgorithm(FourWheelDriveController controller, Gyro gyro) {
        super(controller);

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
        SmartDashboard.putData("Rotation PID Controller", rotationPIDController);
    }

    /**
     * {@inheritDoc} This also uses the gyro to keep the robot going in a
     * straight line.
     */
    public void driveCartesian(double x, double y, double rotation) {
        driveFieldCartesianImplPID(x, y, rotation, gyro.getAngle() - gyroOffset);
    }

    /**
     * Moves the robot forward and sideways at the specified speeds.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     *
     */
    public void driveFieldCartesian(double x, double y) {
        driveFieldCartesian(x, y, 0);
    }

    /**
     * Moves the robot forward and sideways while rotating at the specified
     * speeds. This moves the robot relative to the field.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     * @param rotation the speed to rotate at while moving (negative =
     * clockwise, positive = counterclockwise)
     */
    public void driveFieldCartesian(double x, double y, double rotation) {
        driveFieldCartesianImplPID(x, y, rotation, gyro.getAngle());
    }

    /**
     * Drive based on the specified joystick using the x and y and twist axes.
     *
     * @param stick the joystick to use
     */
    public void driveFieldCartesian(GenericHID stick) {
        driveFieldCartesian(stick.getX(), stick.getY(), stick.getTwist());
    }

    public void driveFieldOrientationCartesian(double x, double y, double orientation) {
        driveFieldOrientationCartesian(x, y, orientation, 1.0);
    }

    public void driveFieldOrientationCartesian(double x, double y, double orientation, double maxRotationSpeed) {
        if (!rotationPIDController.isEnable() || rotationPIDController.getSetpoint() != orientation) {
            rotationPIDController.setSetpoint(orientation);
            rotationPIDController.enable();
        }
        driveFieldCartesianImplNoPID(x, y, rotationSpeedPID > maxRotationSpeed ? maxRotationSpeed : rotationSpeedPID, gyro.getAngle());
    }

    /**
     * Private implementation of field-oriented cartesian mecanum driving that
     * accounts for PID orientation correction.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     * clockwise, positive = counterclockwise)
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldCartesianImplPID(double x, double y, double rotation, double gyroAngle) {
        rotation = getRotationPID(rotation);
        driveFieldCartesianImplNoPID(x, y, rotation, gyroAngle);
    }

    /**
     * Private implementation of field-oriented cartesian mecanum driving that
     * does not account for PID orientation correction.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     * clockwise, positive = counterclockwise)
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldCartesianImplNoPID(double x, double y, double rotation, double gyroAngle) {
        // Compenstate for gyro angle.
        double rotated[] = DriveUtils.rotateVector(x, y, gyroAngle);
        x = rotated[0];
        y = rotated[1];

        super.driveCartesian(x, y, rotation);
    }

    public void driveFieldOrientationPolar(double magnitude, double direction, double orientation) {
        if (!rotationPIDController.isEnable() || rotationPIDController.getSetpoint() != orientation) {
            rotationPIDController.setSetpoint(orientation);
            rotationPIDController.enable();
        }
        driveFieldPolarImplNoPID(magnitude, direction, rotationSpeedPID, gyro.getAngle());
    }

    /**
     * Drives the robot at the specified speed in the direction specified as an
     * angle in radians. This theoretically takes into account the gyro
     * correction, but it has not been tested because we do not use it.
     *
     * @param magnitude the speed that the robot should drive in a given
     * direction.
     * @param direction the direction the robot should drive in radians,
     * independent of rotation
     */
    public void driveFieldPolar(double magnitude, double direction) {
        driveFieldPolar(magnitude, direction, 0);
    }

    /**
     * Drives the robot at the specified speed in the direction specified as an
     * angle in radians. This theoretically takes into account the gyro
     * correction, but it has not been tested because we do not use it.
     *
     * @param magnitude the speed that the robot should drive in a given
     * direction.
     * @param direction the direction the robot should drive in radians,
     * independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     * independent of the magnitude or direction. [-1.0..1.0]
     */
    public void driveFieldPolar(double magnitude, double direction, double rotation) {
        driveFieldPolarImplPID(magnitude, direction, rotation, gyro.getAngle());
    }

    /**
     * Private implementation of field-oriented polar mecanum driving that
     * accounts for PID orientation correction.
     *
     * @param magnitude the speed that the robot should drive in a given
     * direction.
     * @param direction the direction the robot should drive in radians,
     * independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     * independent of the magnitude or direction. [-1.0..1.0]
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldPolarImplPID(double magnitude, double direction, double rotation, double gyroAngle) {
        rotation = getRotationPID(rotation);
        driveFieldPolarImplNoPID(magnitude, direction, rotation, gyroAngle);
    }

    /**
     * Private implementation of field-oriented polar mecanum driving that does
     * not account for PID orientation correction.
     *
     * @param magnitude the speed that the robot should drive in a given
     * direction.
     * @param direction the direction the robot should drive in radians,
     * independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     * independent of the magnitude or direction. [-1.0..1.0]
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldPolarImplNoPID(double magnitude, double direction, double rotation, double gyroAngle) {
        direction += gyroAngle;
        drivePolar(magnitude, direction, rotation);
    }

    public boolean rotateTo(double angle) {
        return rotateTo(angle, 1.0);
    }

    public boolean rotateTo(double angle, double maxRotationSpeed) {
        driveFieldOrientationCartesian(0, 0, angle, maxRotationSpeed);
        return rotationPIDController.onTarget();
    }

    /**
     * Moves the robot sideways at the specified speed.
     *
     * @param speed The speed and direction to crab (negative = left, positive =
     * right)
     */
    public void crab(double speed) {
        driveCartesian(speed, 0);
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

    public void setPID(double p, double i, double d) {
        rotationPIDController.setPID(p, i, d);
    }

    public void setPID(double p, double i, double d, double f) {
        rotationPIDController.setPID(p, i, d, f);
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

    public double getGyroAngle() {
        return gyro.getAngle();
    }
}
