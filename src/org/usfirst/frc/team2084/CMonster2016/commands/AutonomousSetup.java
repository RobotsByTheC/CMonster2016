/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Base command that runs before any autonomous mode. It homes the arm and
 * resets the gyro.
 * 
 * @author Ben Wolsieffer
 */
public class AutonomousSetup extends CommandGroup {

    public AutonomousSetup() {
        addSequential(new HomeArm());
        addSequential(new ResetGyro());
    }
}
