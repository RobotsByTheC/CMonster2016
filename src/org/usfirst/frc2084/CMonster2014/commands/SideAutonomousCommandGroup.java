/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * DOES NOT WORK! DO NOT USE!
 */
public class SideAutonomousCommandGroup extends CommandGroup {

    public SideAutonomousCommandGroup(boolean leftGoal) {
        addSequential(new ResetGyroComand());
        addSequential(new SideAutonomousDriveCommand(leftGoal));
        addSequential(new FrontAutonomousDriveCommand(!leftGoal));
        addSequential(new SweeperEjectCommand());
    }
}
