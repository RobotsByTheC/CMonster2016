/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.DelegateCommand;

import org.usfirst.frc.team2084.CMonster2015.Robot;

/**
 *
 * @author Ben Wolsieffer
 */
public class RotateCommand extends DelegateCommand {

	public RotateCommand(double angle, double timeout) {
		super(new RotateToCommand(Robot.driveSubsystem.getMecanumDriveAlgorithm().getGyroAngle() + angle, timeout));
	}
}
