/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.drive.processors;

/**
 *
 * @author Ben Wolsieffer
 */
@FunctionalInterface
public interface Processor {

	double process(double value, double timeStep);
}
