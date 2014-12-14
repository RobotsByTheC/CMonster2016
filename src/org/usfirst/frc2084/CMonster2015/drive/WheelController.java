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
public class WheelController {

	protected SpeedController[] motors;
	private double[] invertedMotor;

	/**
	 * Creates a new {@link WheelController} with one motor.
	 *
	 * @param motor the {@link SpeedController} that powers the wheel
	 */
	public WheelController(SpeedController motor) {
		this(new SpeedController[] { motor });
	}

	/**
	 * Creates a new {@link WheelController} with two motors.
	 *
	 * @param motor the {@link SpeedController} that powers the wheel
	 * @param motor2 the second {@link SpeedController} that powers the wheel
	 */
	public WheelController(SpeedController motor, SpeedController motor2) {
		this(new SpeedController[] { motor, motor2 });
	}

	/**
	 * Creates a new {@link WheelController} with three motors.
	 *
	 * @param motor the {@link SpeedController} that powers the wheel
	 * @param motor2 the second {@link SpeedController} that powers the wheel
	 * @param motor3 the third {@link SpeedController} that powers the wheel
	 */
	public WheelController(SpeedController motor, SpeedController motor2, SpeedController motor3) {
		this(new SpeedController[] { motor, motor2, motor3 });
	}

	/**
	 * Creates a new {@link WheelController} with an arbitrary number of motors.
	 * This constructor is only necessary if one wheel has more than 3 motors
	 * (which seems unlikely), otherwise use the 1, 2, or 3 parameter versions
	 * of the constructor.
	 *
	 * @param motors the array of {@link SpeedController} that powers the wheel
	 */
	public WheelController(SpeedController[] motors) {
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
