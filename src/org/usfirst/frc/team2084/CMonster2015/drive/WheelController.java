/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Controls a single wheel with an arbitrary number of motors. In many cases
 * this will actually control a single wheel, but in designs where multiple
 * wheels are connected to the same gearbox (such as most skid steer systems),
 * it will actually control three to four wheels.
 *
 * @author Ben Wolsieffer
 */
public class WheelController<S extends SpeedController> {

	protected S[] motors;
	private double inverted = 1;

	/**
	 * Creates a new {@link WheelController} with an arbitrary number of motors.
	 *
	 * @param motors the array of speed controllers that power the wheel
	 */
	@SafeVarargs
	public WheelController(S... motors) {
		this.motors = motors;
	}

	/**
	 * Sets the speed of the wheel. For a basic wheel this just sets the power
	 * to the motor but this class can be extended for more complex
	 * functionality.
	 *
	 * @param speed the wheel speed between 1.0 and -1.0
	 */
	public void set(double speed) {
		speed = DriveUtils.limit(speed) * inverted;
		for (int i = 0; i < motors.length; i++) {
			motors[i].set(speed);
		}
	}

	/**
	 * Sets the inversion of all the motors of this wheel.
	 *
	 * @param inverted whether the wheel is inverted
	 */
	public void setInverted(boolean inverted) {
		this.inverted = inverted ? -1 : 1;
	}

	/**
	 * Gets whether the wheel motors are inverted.
	 * 
	 * @return whether the wheel is inverted
	 */
	public boolean isInverted() {
		return inverted == -1;
	}
}
