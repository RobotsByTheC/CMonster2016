/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * @author Ben Wolsieffer
 */
public abstract class EncoderWheelController<S extends SpeedController> extends WheelController<S> {

	/**
	 * 
	 */
	@SafeVarargs
	public EncoderWheelController(S... motors) {
		super(motors);
	}

	public abstract void rotateTo(double position);

	public abstract double getPosition();
}
