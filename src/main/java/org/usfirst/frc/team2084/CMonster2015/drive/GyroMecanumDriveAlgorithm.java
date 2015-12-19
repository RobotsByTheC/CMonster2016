/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import org.usfirst.frc.team2084.CMonster2015.Gyro;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PIDController;
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
public class GyroMecanumDriveAlgorithm<S extends WheelController<?>> extends MecanumDriveAlgorithm<S> {

    /**
     * The {@link Gyro} that the {@link GyroMecanumDriveAlgorithm} uses for
     * field-oriented driving and keeping the correct orientation.
     */
    protected final Gyro gyro;

    /**
     * The rotation speed below which the heading PID controller is enabled.
     * When the rotation speed increases above this value the controller is
     * disabled to allow the robot to turn.
     */
    public final double ROTATION_DEADBAND = 0.05;

    /**
     * The output of the heading PID controller.
     */
    private volatile double headingPID = 0.0;

    /**
     * Stores the heading offset used for robot centric driving.
     */
    private double headingOffset = 0.0;

    /**
     * Flags that stores whether the gyro should be enabled.
     */
    private boolean gyroEnabled = true;

    private double headingInverted = 1.0;

    /**
     * PID controller that maintains the orientation of the robot using the
     * gyro.
     */
    private final PIDController headingPIDController;

    /**
     * Creates a new {@link GyroMecanumDriveAlgorithm} that controls the
     * specified {@link FourWheelDriveController}.
     *
     * @param controller the {@link FourWheelDriveController} to control
     * @param gyro the {@link SynchronizedRadianGyro} to use for orientation
     *            correction and field-oriented driving
     * @param headingPIDConstants the {@link PIDConstants} to use for heading
     *            control
     * @param headingTolerance the tolerance (in radians) to consider as on
     *            target
     */
    public GyroMecanumDriveAlgorithm(FourWheelDriveController<S> controller, Gyro gyro,
            PIDConstants headingPIDConstants, double headingTolerance) {
        super(controller);
        this.gyro = gyro;

        headingPIDController = DriveUtils.createPIDControllerFromConstants(headingPIDConstants,
                this::getHeading, (o) -> headingPID = -o);
        // (o) -> headingPID = Math.abs(o) > 0.9 ? 0.0 : -o);
        headingPIDController.setAbsoluteTolerance(headingTolerance);
        headingPIDController.setInputRange(-Math.PI, Math.PI);
        headingPIDController.setContinuous(true);
        SmartDashboard.putData("Heading PID Controller", headingPIDController);

    }

    /**
     * {@inheritDoc} This also uses the gyro to keep the robot going in a
     * straight line.
     */
    @Override
    public void driveCartesian(double x, double y, double rotation) {
        driveFieldCartesianImplPID(x, y, rotation, getHeading() - headingOffset);
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
     *            clockwise, positive = counterclockwise)
     */
    public void driveFieldCartesian(double x, double y, double rotation) {
        driveFieldCartesianImplPID(x, y, rotation, getHeading());
    }

    /**
     * Drive based on the specified joystick using the x and y and twist axes.
     *
     * @param stick the joystick to use
     */
    public void driveFieldCartesian(GenericHID stick) {
        driveFieldCartesian(stick.getX(), stick.getY(), stick.getTwist());
    }

    /**
     * Drives the robot at the specified x and y speeds relative to the field
     * while maintaining the specified heading.
     * 
     * @param x the x speed
     * @param y the y speed
     * @param heading the heading to maintain
     */
    public void driveFieldHeadingCartesian(double x, double y, double heading) {
        driveFieldHeadingCartesian(x, y, heading, 1.0);
    }

    /**
     * Drives the robot at the specified x and y speeds relative to the field
     * while maintaining the specified heading. It also limits the maximum
     * rotation speed.
     * 
     * @param x the x speed
     * @param y the y speed
     * @param heading the heading to maintain
     * @param maxRotationSpeed the maximum speed rotation speed
     */
    public void driveFieldHeadingCartesian(double x, double y, double heading,
            double maxRotationSpeed) {
        if (!headingPIDController.isEnable() || headingPIDController.getSetpoint() != heading) {
            headingPIDController.setSetpoint(heading);
            headingPIDController.enable();
        }
        double headingPIDSign = headingPID > 0 ? 1 : -1;
        driveFieldCartesianImplNoPID(x, y,
                Math.abs(headingPID) > maxRotationSpeed ? maxRotationSpeed
                        * headingPIDSign : headingPID,
                getHeading());
    }

    /**
     * Private implementation of field-oriented cartesian mecanum driving that
     * accounts for PID heading correction.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     *            clockwise, positive = counterclockwise)
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldCartesianImplPID(double x, double y, double rotation, double gyroAngle) {
        rotation = getRotationPID(rotation);
        driveFieldCartesianImplNoPID(x, y, rotation, gyroAngle);
    }

    /**
     * Private implementation of field-oriented cartesian mecanum driving that
     * does not account for PID heading correction.
     *
     * @param x the forward speed (negative = backward, positive = forward)
     * @param y the sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     *            clockwise, positive = counterclockwise)
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldCartesianImplNoPID(double x, double y, double rotation,
            double gyroAngle) {
        // Compensate for gyro angle.
        double rotated[] = DriveUtils.rotateVector(x, y, gyroAngle);
        x = rotated[0];
        y = rotated[1];

        super.driveCartesian(x, y, rotation);
    }

    /**
     * Drives the robot at the specified speed and in the specified direction,
     * while maintaining the specified heading.
     * 
     * @param magnitude the movement speed
     * @param direction the movement direction
     * @param heading the heading to maintain
     * @return true when the heading is on target
     */
    public boolean driveFieldHeadingPolar(double magnitude, double direction, double heading) {
        return driveFieldHeadingPolar(magnitude, direction, heading, 1.0);
    }

    /**
     * Drives the robot at the specified speed and in the specified direction,
     * while maintaining the specified heading. It also limits the maximum
     * rotation speed.
     * 
     * @param magnitude the movement speed
     * @param direction the movement direction
     * @param heading the heading to maintain
     * @param maxRotationSpeed the maximum speed rotation speed
     * @return true when the heading is on target
     */
    public boolean driveFieldHeadingPolar(double magnitude, double direction, double heading,
            double maxRotationSpeed) {
        if (!headingPIDController.isEnable() || headingPIDController.getSetpoint() != heading) {
            headingPIDController.setSetpoint(heading);
            headingPIDController.enable();
        }
        double headingPIDSign = headingPID > 0 ? 1 : -1;
        driveFieldPolarImplNoPID(magnitude, direction,
                Math.abs(headingPID) > maxRotationSpeed ? maxRotationSpeed
                        * headingPIDSign : headingPID,
                getHeading());
        return headingPIDController.onTarget();
    }

    /**
     * Drives the robot at the specified speed in the direction specified as an
     * angle in radians. This theoretically takes into account the gyro
     * correction, but it has not been tested because we do not use it.
     *
     * @param magnitude the speed that the robot should drive in a given
     *            direction.
     * @param direction the direction the robot should drive in radians,
     *            independent of rotation
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
     *            direction.
     * @param direction the direction the robot should drive in radians,
     *            independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     *            independent of the magnitude or direction. [-1.0..1.0]
     */
    public void driveFieldPolar(double magnitude, double direction, double rotation) {
        driveFieldPolarImplPID(magnitude, direction, rotation, getHeading());
    }

    /**
     * Private implementation of field-oriented polar mecanum driving that
     * accounts for PID heading correction.
     *
     * @param magnitude the speed that the robot should drive in a given
     *            direction.
     * @param direction the direction the robot should drive in radians,
     *            independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     *            independent of the magnitude or direction. [-1.0..1.0]
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldPolarImplPID(double magnitude, double direction, double rotation,
            double gyroAngle) {
        rotation = getRotationPID(rotation);
        driveFieldPolarImplNoPID(magnitude, direction, rotation, gyroAngle);
    }

    /**
     * Private implementation of field-oriented polar mecanum driving that does
     * not account for PID heading correction.
     *
     * @param magnitude the speed that the robot should drive in a given
     *            direction.
     * @param direction the direction the robot should drive in radians,
     *            independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     *            independent of the magnitude or direction. [-1.0..1.0]
     * @param gyroAngle the current angle reading from the gyro
     */
    private void driveFieldPolarImplNoPID(double magnitude, double direction, double rotation,
            double gyroAngle) {
        direction += gyroAngle;
        drivePolar(magnitude, direction, rotation);
    }

    /**
     * Moves the robot sideways at the specified speed.
     *
     * @param speed The speed and direction to crab (negative = left, positive =
     *            right)
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
        if (gyroEnabled) {
            // If the controller is already enabled, check to see if it should
            // be disabled or kept running. Otherwise check to see if it needs
            // to be enabled.
            if (headingPIDController.isEnable()) {
                // If the rotation rate is greater than the deadband disable the
                // PID controller. Otherwise, return the latest value from the
                // controller.
                if (Math.abs(rotationSpeed) >= ROTATION_DEADBAND) {
                    headingPIDController.disable();
                } else {
                    return headingPID;
                }
            } else {
                // If the rotation rate is less than the deadband, turn on the
                // PID controller and set its setpoint to the current angle.
                if (Math.abs(rotationSpeed) < ROTATION_DEADBAND) {
                    headingOffset = getHeading();
                    headingPIDController.setSetpoint(headingOffset);
                    headingPID = 0;
                    headingPIDController.enable();
                }
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
        headingPIDController.reset();
        // Since the gyro value is now zero, the robot should also try to point
        // in that direction.
        headingPIDController.setSetpoint(0);
        // Reset the output to 0.
        headingPID = 0;
        // Re-enable the controller because it was disabled by calling reset().
        headingPIDController.enable();
    }

    /**
     * Resets the setpoint of the heading PID controller to the current heading.
     */
    public void resetSetpoint() {
        headingPIDController.setSetpoint(getHeading());
        headingPID = 0;
    }

    /**
     * Gets the heading of the robot in radians according to the gyro. This also
     * inverts the value if necessary. This *must* be used to retrieve the gyro
     * heading rather than calling {@link Gyro#getAngle()} to prevent race
     * conditions with the {@link PIDController}.
     * 
     * @return the heading
     */
    public double getHeading() {
        synchronized (this) {
            return DriveUtils.normalizeHeading(gyro.getAngle() * headingInverted);
        }
    }

    /**
     * Sets the heading of the robot. This should be called rather than
     * {@link Gyro#setAngle(double)} to prevent the robot from trying to rotate
     * to this new heading, which is generally not the desired behavior.
     * 
     * @param heading
     */
    public void setHeading(double heading) {
        synchronized (this) {
            gyro.setAngle(DriveUtils.normalizeHeading(heading * headingInverted));
        }
        // This must not be synchronized to avoid deadlock
        resetSetpoint();
    }

    /**
     * Gets the rate of rotation of the robot in radians per second according to
     * the gyro. This also inverts the value if necessary. This *must* be used
     * to retrieve the rotation rate rather than calling {@link Gyro#getRate(s)}
     * to prevent race conditions with the {@link PIDController}.
     * 
     * @return the heading
     */
    public double getRotationRate() {
        synchronized (this) {
            return gyro.getRate() * headingInverted;
        }
    }

    public void setHeadingInverted(boolean inverted) {
        headingInverted = inverted ? -1.0 : 1.0;
    }

    public boolean isHeadingInverted() {
        return headingInverted == -1.0;
    }

    /**
     * Gets the angular speed of the robot in radians/second according to the
     * gyro.
     * 
     * @return the angular speed
     */
    public double getAngularSpeed() {
        synchronized (this) {
            return gyro.getRate() * headingInverted;
        }
    }

    /**
     * Gets whether the robot is facing the direction it should be. This always
     * returns true if the robot is being commanded to spinat a certain rate.
     * 
     * @return true if the robot is on target
     */
    public boolean isHeadingOnTarget() {
        if (headingPIDController.isEnable()) {
            return headingPIDController.onTarget();
        } else {
            return true;
        }
    }

    /**
     * Sets whether the algorithm should use the gyro. It would be disabled if
     * it fails during a match. Methods that require a gyro for basic
     * functionality are not affected by this.
     * 
     * @param enabled whether the gyro should be enabled
     */
    public void setGyroEnabled(boolean enabled) {
        gyroEnabled = enabled;
    }

    /**
     * Gets whether the gyro is enabled.
     * 
     * @see #setGyroEnabled(boolean)
     * @return whether the gyro is enabled
     */
    public boolean isGyroEnabled() {
        return gyroEnabled;
    }

    public double getHeadingError() {
        return headingPIDController.getError();
    }
}
