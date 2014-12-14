/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * CommandGroup that runs the driving command in parallel with the vision
 * command. This needs to be separated into its own command group to allow it to
 * run sequentially after the initial wait at the beginning of
 * {@link FrontAutonomousCommandGroup} and before the sweeper expels the ball.
 *
 * @see FrontAutonomousCommandGroup
 */
public class FrontAutonomousDriveAndVisionCommandGroup extends CommandGroup {

	/**
	 * Instantiates this command group to drive toward the specified wall. The
	 * parameter is simply passed through to the driving command.
	 *
	 * @param leftGoal left goal = true, right goal = false
	 */
	public FrontAutonomousDriveAndVisionCommandGroup(boolean leftGoal) {
		// Run these in parallel so that the DS laptop start processing vision
		// while the robot drives. This works because the vision targets are
		// still visible until the robot is very close to the wall.
		addParallel(new FrontAutonomousDriveCommand(leftGoal));
		addParallel(new WaitForVisionCommand());
	}
}
