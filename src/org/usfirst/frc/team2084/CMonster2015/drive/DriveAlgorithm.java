/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

/**
 * This class acts as a bridge between a HID (joystick, gamepad) and a
 * {@link DriveController}. It takes input and uses it to control the robot
 * through different algorithms. This class cannot be used directly and must be
 * extended to provide functionality. A drive algorithm does not have to worry
 * about where the wheels are in most cases, because this is taken care of by
 * the {@link DriveController}.
 *
 * @see ArcadeDriveAlgorithm
 * @see MecanumDriveAlgorithm
 * @see DriveController
 *
 * @author Ben Wolsieffer
 */
public abstract class DriveAlgorithm<C extends DriveController<?>> {

	protected C controller;

	/**
	 * Creates a new {@link DriveAlgorithm} that controls the specified
	 * {@link DriveController}.
	 *
	 * @param controller the drive controller to use
	 */
	public DriveAlgorithm(C controller) {
		this.controller = controller;
	}

	/**
	 * Stops the robot. This just calls {@link DriveController#stop()}.
	 */
	public void stop() {
		controller.stop();
	}
}
