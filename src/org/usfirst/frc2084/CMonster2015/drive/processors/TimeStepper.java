/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.drive.processors;

import org.usfirst.frc2084.CMonster2015.Utils;

/**
 * @author Ben Wolsieffer
 */
public class TimeStepper {

	private double lastTime;

	public TimeStepper() {
		reset();
	}

	public double step() {
		double currTime = Utils.getTime();
		double step = currTime - lastTime;

		if (step == 0) {
			// Little hack to make sure more than 0 time had passed.
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
			}
			return step();
		}
		else {
			lastTime = currTime;
			return step;
		}
	}

	public void reset() {
		lastTime = Utils.getTime();
	}
}
