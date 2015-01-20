/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.command;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A command that does something once at the beginning but does not finish until
 * a certain time has elapsed.
 *
 * @author Ben Wolsieffer
 */
public abstract class TimedCommand extends Command {

	public TimedCommand(double time) {
		super(time);
	}

	/**
	 * Initialize calls {@link #run()}.
	 */
	@Override
	protected final void initialize() {
		runCommand();
	}

	protected abstract void runCommand();

	@Override
	protected final void execute() {
	}

	/**
	 * This type of command never finishes but instead uses a timeout to end.
	 *
	 * @return false
	 */
	@Override
	protected boolean isFinished() {
		return false;
	}

}
