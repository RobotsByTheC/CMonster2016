/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.drive.Location;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * An autonomous mode that picks up the recycling container from behind and
 * drives over the scoring platform into the auto zone.
 * 
 * @author Ben Wolsieffer
 */
public class ContainerPickupAutoZoneAutonomousCommand extends CommandGroup {

    public ContainerPickupAutoZoneAutonomousCommand() {
        addParallel(new RaiseContainerHookCommand());
        addSequential(new WaitCommand(0.3));
        addSequential(new DriveToLocationCommand(new Location(0, 2), 0));
    }
}
