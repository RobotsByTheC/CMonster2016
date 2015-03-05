/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous mode that lifts the recycling container, tote lifter (if not
 * noodle loading) and floor tab.
 * 
 * @author Ben Wolsieffer
 */
public class ContainerAutonomousCommand extends CommandGroup {

    /**
     * Creates a {@link ContainerAutonomousCommand} that raises the container
     * for a noodle or all the way.
     * 
     * @param noodle whether to prepare to load a noodle
     */
    public ContainerAutonomousCommand(boolean noodle) {
        // Tell the gyro that we are facing backwards.
        addSequential(new SetHeadingCommand(Math.PI));
        // Raise the floor tab.
        addSequential(new CloseToteGateCommand());

        if (noodle) {
            // Raise the container hook for a noodle.
            addSequential(new RaiseContainerHookCommand(1.5));
        } else {
            // Raise the container hook with the container on it.
            addSequential(new RaiseContainerHookCommand());

            // Raise the tote lifter to be ready to accept a tote.
            addSequential(new RaiseToteLifterCommand());
        }
    }
}
