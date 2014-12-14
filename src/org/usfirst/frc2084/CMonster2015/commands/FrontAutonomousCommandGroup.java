/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Runs the autonomous mode where the robot drives forward and expels the ball
 * into the front of the low goal. It is called front autonomous because there
 * originally was going to be a second mode where it would score on the side of
 * the goal ({@link SideAutonomousCommandGroup}), but it didn't work well and
 * was deemed unnecessary.
 */
public class FrontAutonomousCommandGroup extends CommandGroup {

	/**
	 * The time to wait before starting vision and driving. This is necessary to
	 * work around the hot goal targets sometimes not changing right away at the
	 * beginning of the match. <a
	 * href="http://www.chiefdelphi.com/forums/showthread.php?t=127714">See this
	 * CD thread for more information.</a> Stupid FMS! if they have not changed
	 * after this, we'll just screw them and do our autonomous without vision.
	 */
	public static final double WAIT_TIME = 0.5;

	/**
	 * Instantiates this command group to drive toward the specified wall. The
	 * parameter is simply passed through to the driving and vision command
	 * group.
	 *
	 * @param leftGoal left goal = true, right goal = false
	 */
	public FrontAutonomousCommandGroup(boolean leftGoal) {
		// Make sure to reset the gyro at the beginning of autonomous to correct
		// for drift that occured whie waiting for the match to start.
		addSequential(new ResetGyroCommand());
		// Wait to make sure the hot goals have had time to change.
		addSequential(new WaitCommand(WAIT_TIME));
		// Start driving and processing vision in parallel. The only easy way to
		// wait and then run two tasks in parallel in the command based system
		// is to put them into their own CommandGroup.
		addSequential(new FrontAutonomousDriveAndVisionCommandGroup(leftGoal));
		// After the robot hits the wall and vision finishes (the latter should
		// happen first), expel the ball.
		addSequential(new SweeperEjectCommand());
	}
}
