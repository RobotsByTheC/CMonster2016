/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Autonomous mode that drives under the low bar. This possibly would work for
 * other defenses too.
 * 
 * @author Ben Wolsieffer
 */
public class LowBarAutonomous extends CommandGroup {

    public LowBarAutonomous() {
        addSequential(new AutonomousSetup());
        addParallel(new SetArmAngle(Math.toRadians(15)));
        addSequential(new DriveHeading(0, 0.8, 3.3));
    }
}