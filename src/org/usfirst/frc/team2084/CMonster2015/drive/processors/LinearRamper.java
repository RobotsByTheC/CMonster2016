/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive.processors;

import org.usfirst.frc.team2084.CMonster2015.Utils;

/**
 *
 * @author Ben Wolsieffer
 */
public class LinearRamper implements Processor {

	public static enum Type {
		UP, DOWN, UP_DOWN
	}

	private final Type type;
	private final double rampRate;
	private double lastValue;
	private double lastTime;

	public LinearRamper(double rampRate, Type type) {
		this.rampRate = Math.abs(rampRate);
		this.type = type;
		reset();
	}

	@Override
	public double process(double value) {
		double currTime = Utils.getTime();
		double elapsedTime = currTime - lastTime;
		lastTime = currTime;

		double delta = value - lastValue;
		double maxDelta = rampRate * elapsedTime;
		double output = value;
		if (Math.abs(delta) > maxDelta) {
			delta = maxDelta * delta < 0 ? -1 : 1;
			output = lastValue + delta;
			if (type == Type.UP_DOWN) {
				output = value;
			} else if (delta < 0) {
				if (output < 0) {
					if (type == Type.UP) {
						output = value;
					}
				} else {
					if (type == Type.DOWN) {
						output = value;
					}
				}
			} else {
				if (output < 0) {
					if (type == Type.DOWN) {
						output = value;
					}
				} else {
					if (type == Type.UP) {
						output = value;
					}
				}
			}
		}
		lastValue = output;
		return output;
	}

	public void reset() {
		lastValue = 0;
		lastTime = Utils.getTime();
	}
}
