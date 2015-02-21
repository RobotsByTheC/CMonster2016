/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.command;

/**
 * A command group that runs the specified commands sequentially.
 * 
 * @author Ben Wolsieffer
 */
public class SequentialCommandGroup extends CommandGroup {

    /**
     * Creates a {@link CommandGroup} that runs the specified commands
     * sequentially.
     * 
     * @param commands the commands to run
     */
    public SequentialCommandGroup(Command... commands) {
        for (Command command : commands) {
            addSequential(command);
        }
    }
}
