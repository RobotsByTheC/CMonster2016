/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.drive.processors;


/**
 * Generates inertia for an input value. This makes the robot respond faster to
 * quick changes in input. Functionally, it acts like a PD controller with a
 * {@code P=1.0}, {@code D=inertiaGain} and {@code error=value}. This idea was
 * taken from Team 254 (The Cheesy Poofs), but I implemented it somewhat
 * differently.
 *
 * @author Ben Wolsieffer
 */
public class InertiaGenerator implements Processor {

	public double lastValue = 0.0;

	private final double inertiaGain;

	public InertiaGenerator(double inertiaGain) {
		this.inertiaGain = inertiaGain;
	}

	@Override
	public double process(double value, double timeStep) {
		// Get the difference between current and previous values
		double output = value - lastValue;
		// Normalize based on elapsed time
		output /= timeStep;
		// Multiply the normalized delta by the gain
		output *= inertiaGain;
		// Update the last value
		lastValue = value;
		return value + output;
	}

}
