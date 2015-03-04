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
 * drives over the scoring platform into the auto zone.
 * 
 * @author Ben Wolsieffer
 */
public class ContainerAutoZoneAutonomousCommand extends CommandGroup {

    public ContainerAutoZoneAutonomousCommand() {
        addParallel(new ContainerAutonomousCommand());
        // Wait for the container to get off the ground
        addSequential(new WaitCommand(1));
        // Drive into the auto zone
        addSequential(new AutoZoneDriveCommand());
    }
}
