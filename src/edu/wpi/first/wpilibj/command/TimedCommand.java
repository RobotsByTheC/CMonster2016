/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.command;

/**
 * A command that does something once at the beginning but does not finish until
 * a certain time has elapsed.
 *
 * @author Ben Wolsieffer
 */
public abstract class TimedCommand extends WaitCommand {

    /**
     * Creates a {@link TimedCommand} that last for the specified number of
     * seconds.
     * 
     * @param time the length of the command in seconds
     */
    public TimedCommand(double time) {
        super(time);
    }

    /**
     * Does nothing.
     */
    @Override
    protected final void execute() {
    }

}
