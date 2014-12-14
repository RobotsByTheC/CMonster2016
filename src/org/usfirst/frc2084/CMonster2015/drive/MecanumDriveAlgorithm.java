/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Ben Wolsieffer
 */
public class MecanumDriveAlgorithm extends DriveAlgorithm<FourWheelDriveController<WheelController>> {

	public MecanumDriveAlgorithm(FourWheelDriveController<WheelController> controller) {
		super(controller);
	}

	/**
	 * Moves the robot forward and sideways at the specified speeds. This moves
	 * the robot relative to the robot's current orientation.
	 *
	 * @param x The forward speed (negative = backward, positive = forward)
	 * @param y The sideways (crab) speed (negative = left, positive = right)
	 */
	public void driveCartesian(double x, double y) {
		driveCartesian(x, y, 0);
	}

	/**
	 * Moves the robot forward and sideways while rotating at the specified
	 * speeds. This moves the robot relative to the robot's current orientation.
	 *
	 * @param x The forward speed (negative = backward, positive = forward)
	 * @param y The sideways (crab) speed (negative = left, positive = right)
	 * @param rotation The speed to rotate at while moving (negative =
	 *            clockwise, positive = counterclockwise)
	 */
	public void driveCartesian(double x, double y, double rotation) {
		// Send debugging values.
		SmartDashboard.putNumber("Mecanum X", x);
		SmartDashboard.putNumber("Mecanum Y", y);
		SmartDashboard.putNumber("Mecanum Rotation", rotation);

		double wheelSpeeds[] = new double[4];
		wheelSpeeds[0] = x + y + rotation; // Front left speed
		wheelSpeeds[1] = -x + y - rotation; // Front right speed
		wheelSpeeds[2] = -x + y + rotation; // Rear left speed
		wheelSpeeds[3] = x + y - rotation; // Rear right speed

		DriveUtils.normalize(wheelSpeeds);

		controller.drive(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
	}

	/**
	 * Real implementation of polar mecanum driving. This method does not
	 * include gyro orientation correction and is only called from with this
	 * class.
	 *
	 * @param magnitude the speed that the robot should drive in a given
	 *            direction.
	 * @param direction the direction the robot should drive in radians,
	 *            independent of rotation
	 * @param rotation the rate of rotation for the robot that is completely
	 *            independent of the magnitude or direction. [-1.0..1.0]
	 */
	public void drivePolar(double magnitude, double direction, double rotation) {
		// Send debugging values.
		SmartDashboard.putNumber("Mecanum Magnitude", magnitude);
		SmartDashboard.putNumber("Mecanum Direction", direction);
		SmartDashboard.putNumber("Mecanum Rotation", rotation);

		// Normalized for full power along the Cartesian axes.
		magnitude = DriveUtils.limit(magnitude) * Math.sqrt(2.0);
		// The rollers are at 45 degree (pi/4 radian) angles.
		direction += Math.PI / 4.0;
		double cosD = Math.cos(direction);
		double sinD = Math.sin(direction);

		double wheelSpeeds[] = new double[4];
		wheelSpeeds[0] = (sinD * magnitude + rotation);
		wheelSpeeds[1] = (cosD * magnitude - rotation);
		wheelSpeeds[2] = (cosD * magnitude + rotation);
		wheelSpeeds[3] = (sinD * magnitude - rotation);

		DriveUtils.normalize(wheelSpeeds);

		controller.drive(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
	}
}
