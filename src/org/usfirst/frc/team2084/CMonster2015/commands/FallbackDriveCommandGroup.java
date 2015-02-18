/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class FallbackDriveCommandGroup extends CommandGroup {

    public FallbackDriveCommandGroup() {
        addSequential(new SetGyroEnabledCommand(false));
        addSequential(new SetEncodersEnabledCommand(false));
        addSequential(new MecanumDriveCommand(false));
    }
}
