/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.SynchronizedRadianGyro;

/**
 * @author Ben Wolsieffer
 */
public class EncoderGyroMecanumDriveAlgorithm<S extends EncoderWheelController<?>> extends GyroMecanumDriveAlgorithm<S> {

	/**
	 * @param controller
	 * @param gyro
	 */
	public EncoderGyroMecanumDriveAlgorithm(FourWheelDriveController<S> controller, SynchronizedRadianGyro gyro) {
		super(controller, gyro);
	}

}
