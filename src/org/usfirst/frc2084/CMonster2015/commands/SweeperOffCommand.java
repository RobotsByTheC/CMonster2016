/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2084.CMonster2015.Robot;

/**
 * Command that turns on the sweeper motor off.
 *
 * @author Ben Wolsieffer
 */
public class SweeperOffCommand extends Command {
	public SweeperOffCommand() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
		requires(Robot.sweeperSubsystem);
		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
	}

	/**
	 * Does nothing.
	 */
	protected void initialize() {
	}

	/**
	 * Only runs once and turns the sweeper motor off.
	 */
	protected void execute() {
		Robot.sweeperSubsystem.off();
	}

	/**
	 * This command only needs to run once so this always returns true.
	 *
	 * @return true
	 */
	protected boolean isFinished() {
		return true;
	}

	/**
	 * Does nothing.
	 */
	protected void end() {
	}

	/**
	 * Does nothing.
	 */
	protected void interrupted() {
	}
}