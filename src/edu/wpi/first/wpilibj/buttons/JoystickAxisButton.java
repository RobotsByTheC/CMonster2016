/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.buttons;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A button that is triggered when a joystick axis is greater than a certain
 * value (in positive and negative directions).
 *
 * @author Ben Wolsieffer
 */
public class JoystickAxisButton extends Button {

	public static final double DEFAULT_THRESHOLD = 0.7;

	private final GenericHID joystick;
	private final int axis;
	private final double threshold;

	/**
	 * Create a new {@link JoystickAxisButton} on the specified joystick and
	 * axis, using {@link #DEFAULT_THRESHOLD}
	 *
	 * @param joystick the joystick to use
	 * @param axis which axis of the joystick to use
	 */
	public JoystickAxisButton(GenericHID joystick, int axis) {
		this(joystick, axis, DEFAULT_THRESHOLD);
	}

	/**
	 * Create a new {@link JoystickAxisButton} on the specified joystick and
	 * axis, with the specified threshold.
	 *
	 * @param joystick the joystick to use
	 * @param axis which axis of the joystick to use
	 * @param threshold the threshold above which the button is considered
	 *            pressed
	 */
	public JoystickAxisButton(GenericHID joystick, int axis, double threshold) {
		this.joystick = joystick;
		this.axis = axis;
		this.threshold = threshold;
	}

	public boolean get() {
		double axisValue = joystick.getRawAxis(axis);
		return threshold < 0 ? axisValue < threshold : axisValue > threshold;
	}
}
