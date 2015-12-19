/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous mode that lifts the recycling container, tote lifte rand floor
 * tab.
 * 
 * @author Ben Wolsieffer
 */
public class ContainerAutonomousCommand extends CommandGroup {

    /**
     * Creates a {@link ContainerAutonomousCommand} that raises the container.
     */
    public ContainerAutonomousCommand() {
        // Tell the gyro that we are facing backwards.
        addSequential(new SetHeadingCommand(Math.PI));
        // Raise the floor tab.
        addSequential(new CloseToteGateCommand());

        // Raise the container hook with the container on it.
        addSequential(new RaiseContainerHookCommand());

        // Raise the tote lifter to be ready to accept a tote.
        addSequential(new RaiseToteLifterCommand());

    }
}
