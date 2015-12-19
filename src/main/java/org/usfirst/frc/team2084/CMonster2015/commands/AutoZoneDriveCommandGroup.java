/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Drives the robot into the auto zone and rotates it 90 degrees so it fits
 * better.
 * 
 * @author Ben Wolsieffer
 */
public class AutoZoneDriveCommandGroup extends CommandGroup {

    public AutoZoneDriveCommandGroup() {
        // Lots of magic numbers...
        addSequential(new DriveHeadingCommand(0, 0.5, 1.33, Math.PI / 2, 0.2, 6.0));
    }
}
