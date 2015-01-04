/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.drive;

import java.util.Arrays;

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
	private double[] invertedMotor;

	/**
	 * Creates a new {@link WheelController} with an arbitrary number of motors.
	 *
	 * @param motors the array of speed controllers that power the wheel
	 */
	@SafeVarargs
	public WheelController(S... motors) {
		this.motors = motors;
		invertedMotor = new double[motors.length];
		Arrays.fill(invertedMotor, 1);
	}

	/**
	 * Sets the speed of the wheel. For a basic wheel this just sets the power
	 * to the motor but this class can be extended for more complex
	 * functionality.
	 *
	 * @param speed the wheel speed between 1.0 and -1.0
	 */
	public void set(double speed) {
		speed = DriveUtils.limit(speed);
		for (int i = 0; i < motors.length; i++) {
			motors[i].set(speed * invertedMotor[i]);
		}
	}

	/**
	 * Sets the inversion of all the motors of this wheel.
	 *
	 * @param inverted whether the wheel is inverted
	 */
	public void setInverted(boolean inverted) {
		Arrays.fill(invertedMotor, inverted ? -1 : 1);
	}

	/**
	 * Set the inverted state of a motor of the wheel.
	 *
	 * @param motor the motor to get (does nothing for invalid indexes)
	 * @param inverted the inverted state
	 */
	public void setInverted(int motor, boolean inverted) {
		if (motor >= 0 && motor < invertedMotor.length) {
			invertedMotor[motor] = inverted ? -1 : 1;
		}
	}

	/**
	 * Gets whether or not a motor of the wheel is inverted.
	 *
	 * @param motor the motor to get
	 * @return the inverted state (or false if motor index is out of range)
	 */
	public boolean isInverted(int motor) {
		if (motor >= 0 && motor < invertedMotor.length) {
			return invertedMotor[motor] == -1;
		} else {
			return false;
		}
	}
}
