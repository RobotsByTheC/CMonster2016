/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * An autonomous mode that picks up the recycling container from behind and
 * drives to the feeder station to prepare to pick up totes.
 * 
 * @author Ben Wolsieffer
 */
public class ContainerFeederStationAutonomousCommand extends CommandGroup {

    /**
     * Creates a new {@link ContainerFeederStationAutonomousCommand} that drives
     * to the specified station.
     * 
     * @param left drives to the left feeder station if true
     */
    public ContainerFeederStationAutonomousCommand(boolean left) {
        // Start the container raising.
        addParallel(new ContainerAutonomousCommand());
        // Wait for the container to get off the ground
        addSequential(new WaitCommand(1));
        // Drive into the auto zone
        addSequential(new FeederStationDriveCommandGroup(left));
    }
}
