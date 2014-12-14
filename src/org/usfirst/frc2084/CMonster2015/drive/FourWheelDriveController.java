/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drive controller for robots with four wheels. Allows control over each side
 * together or each wheel independently.
 *
 * @author Ben Wolsieffer
 */
public class FourWheelDriveController<W extends WheelController> extends DriveController<W> {

	protected W frontLeftWheel;
	protected W frontRightWheel;
	protected W rearLeftWheel;
	protected W rearRightWheel;

	/**
	 * Creates a new {@link FourWheelDriveController} with the specified
	 * {@link WheelController}s.
	 *
	 * @param frontLeftWheel the front left {@link WheelController}
	 * @param frontRightWheel the front right {@link WheelController}
	 * @param rearLeftWheel the rear left {@link WheelController}
	 * @param rearRightWheel the rear right {@link WheelController}
	 */
	public FourWheelDriveController(W frontLeftWheel, W frontRightWheel, W rearLeftWheel, W rearRightWheel) {
		this.frontLeftWheel = frontLeftWheel;
		this.frontRightWheel = frontRightWheel;
		this.rearLeftWheel = rearLeftWheel;
		this.rearRightWheel = rearRightWheel;
	}

	/**
	 * Set the left and right speeds of the robot using the specified values.
	 * For the {@link FourWheelDriveController}, {@code leftSpeed} sets both
	 * left {@link WheelController}s and {@code rightSpeed} sets both right
	 * {@link WheelController}s.
	 *
	 * @param leftSpeed {@inheritDoc}
	 * @param rightSpeed {@inheritDoc}
	 */
	@Override
	public void drive(double leftSpeed, double rightSpeed) {
		drive(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
	}

	/**
	 * Sets the speeds of all four {@link WheelController}s independently.
	 *
	 * @param frontLeftSpeed the speed of the front left wheel
	 * @param frontRightSpeed the speed of the front right wheel
	 * @param rearLeftSpeed the speed of the rear left wheel
	 * @param rearRightSpeed the speed of the rear right wheel
	 */
	public void drive(double frontLeftSpeed, double frontRightSpeed, double rearLeftSpeed, double rearRightSpeed) {
		frontLeftWheel.set(frontLeftSpeed);
		SmartDashboard.putNumber("Front Left Power", frontLeftSpeed);
		frontRightWheel.set(frontRightSpeed);
		SmartDashboard.putNumber("Front Right Power", frontRightSpeed);
		rearLeftWheel.set(rearLeftSpeed);
		SmartDashboard.putNumber("Rear Left Power", rearLeftSpeed);
		rearRightWheel.set(rearRightSpeed);
		SmartDashboard.putNumber("Rear Right Power", rearRightSpeed);

		// Make sure to feed the watchdog!
		safetyHelper.feed();
	}

	/**
	 * Stops the robot.
	 */
	@Override
	public void stop() {
		frontLeftWheel.set(0);
		rearLeftWheel.set(0);
		frontRightWheel.set(0);
		rearRightWheel.set(0);
	}

	@Override
	public String getDescription() {
		return "Four Wheel Drive Controller";
	}
}
