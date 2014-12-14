/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015;

import edu.wpi.first.wpilibj.Utility;

/**
 * Various generic utility methods.
 *
 * @author Ben Wolsieffer
 */
public final class Utils {

	/**
	 * Gets the time in seconds.
	 *
	 * @return the time in seconds
	 */
	public static double getTime() {
		return Utility.getFPGATime() / 1000000.0;
	}
}
