/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.command;

/**
 * Wraps another {@link Command}. This can be used to create a {@link Command}
 * that specifies parameters for another command. {@link DelegateCommand}s
 * finish when their wrapped command does.
 *
 * @author Ben Wolsieffer
 */
public class DelegateCommand extends Command {

	private final Command command;

	/**
	 * Create a new {@link DelegateCommand} which wraps the specified
	 * {@link Command}.
	 *
	 * @param command the command to wrap around
	 */
	public DelegateCommand(Command command) {
		this.command = command;
	}

	@Override
	protected void initialize() {
		command.start();
	}

	@Override
	protected void execute() {
	}

	@Override
	protected boolean isFinished() {
		return command.isFinished();
	}

	@Override
	protected void end() {
	}

	@Override
	protected void interrupted() {
	}
}
