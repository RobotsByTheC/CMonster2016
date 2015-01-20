/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SynchronizedRadianGyro;
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
	 * The {@link SynchronizedRadianGyro} that the {@link GyroMecanumDriveAlgorithm} uses
	 * for field-oriented driving and keeping the correct orientation.
	 */
	protected final SynchronizedRadianGyro gyro;

	/**
	 * The rotation speed below which the heading PID controller is enabled.
	 * When the rotation speed increases above this value the controller is
	 * disabled to allow the robot to turn.
	 */
	public final double ROTATION_DEADBAND = 0.05;

	/**
	 * The proportional constant of the PID algorithm. This value is multiplied
	 * by the error.
	 */
	public static final double HEADING_P = 0.573;
	/**
	 * The integral constant of the PID algorithm. This value is multiplied by
	 * the sum of the error over time. This is used to make the robot turn more
	 * if it is taking a long time to reach its target.
	 */
	public static final double HEADING_I = 0.0;
	/**
	 * The derivative constant of the PID algorithm. This value is multiplied by
	 * the rate of change of the error. This is used to make the robot respond
	 * to sudden increases in error (ie. being spun by another robot) , but we
	 * are unlikely to use it.
	 */
	public static final double HEADING_D = 0.0;
	/**
	 * The feed-forward constant of the PID algorithm. This value is multiplied
	 * by the setpoint, which we don't need for the heading PID.
	 */
	public static final double HEADING_F = 0.0;

	/**
	 * The proportional constant of the PID algorithm. This value is multiplied
	 * by the error.
	 */
	public static final double ANGULAR_SPEED_P = 0.0;
	/**
	 * The integral constant of the PID algorithm. This value is multiplied by
	 * the sum of the error over time. This is used to make the robot turn more
	 * if it is taking a long time to reach its target.
	 */
	public static final double ANGULAR_SPEED_I = 0.0;
	/**
	 * The derivative constant of the PID algorithm. This value is multiplied by
	 * the rate of change of the error. This is used to make the robot respond
	 * to sudden increases in error, but we are unlikely to use it.
	 */
	public static final double ANGULAR_SPEED_D = 0.0;

	/**
	 * The feedforward constant of the PID algorithm. This is multiplied by the
	 * input to keep the output from lagging behind the joystick.
	 */
	public static final double ANGULAR_SPEED_F = 1.0;

	/**
	 * The maximum angular speed (rad/sec) that the robot can actually spin.
	 * This needs to be empirically calculated as accurately as possible.
	 */
	public static final double MAX_ANGULAR_SPEED = 1.0;

	private double rotationPID = 0.0;
	private double headingOffset = 0.0;

	private boolean gyroEnabled = true;

	/**
	 * {@link http://www.chiefdelphi.com/media/papers/download/1829}
	 */
	private final PIDController headingPIDController;
	private final PIDController angularSpeedPIDController;

	/**
	 * Creates a new {@link GyroMecanumDriveAlgorithm} that controls the
	 * specified {@link FourWheelDriveController}.
	 *
	 * @param controller the {@link FourWheelDriveController} to control
	 * @param gyro the {@link SynchronizedRadianGyro} to use for orientation correction and
	 *            field-oriented driving
	 */
	public GyroMecanumDriveAlgorithm(FourWheelDriveController<S> controller, SynchronizedRadianGyro gyro) {
		super(controller);
		this.gyro = gyro;

		angularSpeedPIDController = new PIDController(ANGULAR_SPEED_P, ANGULAR_SPEED_I, ANGULAR_SPEED_D, ANGULAR_SPEED_F,
				() -> gyro.getRate() / MAX_ANGULAR_SPEED, (output) -> rotationPID = output);
		angularSpeedPIDController.enable();
		SmartDashboard.putData("Angular Speed PID Controller", angularSpeedPIDController);

		headingPIDController = new PIDController(HEADING_P, HEADING_I, HEADING_D, HEADING_F,
				gyro::getAngle, angularSpeedPIDController::setSetpoint);
		SmartDashboard.putData("Heading PID Controller", headingPIDController);

	}

	/**
	 * {@inheritDoc} This also uses the gyro to keep the robot going in a
	 * straight line.
	 */
	@Override
	public void driveCartesian(double x, double y, double rotation) {
		driveFieldCartesianImplPID(x, y, rotation, gyro.getAngle() - headingOffset);
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
		if (!headingPIDController.isEnable() || headingPIDController.getSetpoint() != orientation) {
			headingPIDController.setSetpoint(orientation);
			headingPIDController.enable();
		}
		driveFieldCartesianImplNoPID(x, y, rotationPID > maxRotationSpeed ? maxRotationSpeed : rotationPID, gyro.getAngle());
	}

	/**
	 * Private implementation of field-oriented cartesian mecanum driving that
	 * accounts for PID orientation correction.
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
	 * does not account for PID orientation correction.
	 *
	 * @param x the forward speed (negative = backward, positive = forward)
	 * @param y the sideways (crab) speed (negative = left, positive = right)
	 * @param rotation The speed to rotate at while moving (negative =
	 *            clockwise, positive = counterclockwise)
	 * @param gyroAngle the current angle reading from the gyro
	 */
	private void driveFieldCartesianImplNoPID(double x, double y, double rotation, double gyroAngle) {
		// Compensate for gyro angle.
		double rotated[] = DriveUtils.rotateVector(x, y, gyroAngle);
		x = rotated[0];
		y = rotated[1];

		super.driveCartesian(x, y, rotation);
	}

	public void driveFieldOrientationPolar(double magnitude, double direction, double orientation) {
		if (!headingPIDController.isEnable() || headingPIDController.getSetpoint() != orientation) {
			headingPIDController.setSetpoint(orientation);
			headingPIDController.enable();
		}
		driveFieldPolarImplNoPID(magnitude, direction, rotationPID, gyro.getAngle());
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
		driveFieldPolarImplPID(magnitude, direction, rotation, gyro.getAngle());
	}

	/**
	 * Private implementation of field-oriented polar mecanum driving that
	 * accounts for PID orientation correction.
	 *
	 * @param magnitude the speed that the robot should drive in a given
	 *            direction.
	 * @param direction the direction the robot should drive in radians,
	 *            independent of rotation
	 * @param rotation the rate of rotation for the robot that is completely
	 *            independent of the magnitude or direction. [-1.0..1.0]
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
	 *            direction.
	 * @param direction the direction the robot should drive in radians,
	 *            independent of rotation
	 * @param rotation the rate of rotation for the robot that is completely
	 *            independent of the magnitude or direction. [-1.0..1.0]
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
		return headingPIDController.onTarget();
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
			// be
			// disabled or kept running. Otherwise check to see if it needs to
			// be
			// enabled.
			if (headingPIDController.isEnable()) {
				// If the rotation rate is greater than the deadband disable the
				// PID
				// controller. Otherwise, return the latest value from the
				// controller.
				if (Math.abs(rotationSpeed) >= ROTATION_DEADBAND) {
					headingPIDController.disable();
				} else {
					angularSpeedPIDController.setSetpoint(rotationSpeed);
					return rotationPID;
				}
			} else {
				// If the rotation rate is less than the deadband, turn on the
				// PID
				// controller and set its setpoint to the current angle.
				if (Math.abs(rotationSpeed) < ROTATION_DEADBAND) {
					headingOffset = gyro.getAngle();
					headingPIDController.setSetpoint(headingOffset);
					headingPIDController.enable();
				}
			}
		}
		// Unless told otherwise, return the rate that was passed in.
		return rotationSpeed;
	}

	public void setPID(double p, double i, double d) {
		headingPIDController.setPID(p, i, d);
	}

	public void setPID(double p, double i, double d, double f) {
		headingPIDController.setPID(p, i, d, f);
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
		// Renable the controller because it was disabled by calling reset().
		headingPIDController.enable();
	}

	public double getGyroAngle() {
		return gyro.getAngle();
	}

	public double getGyroRate() {
		return gyro.getRate();
	}

	public void setGyroEnabled(boolean enabled) {
		gyroEnabled = enabled;
	}
}
