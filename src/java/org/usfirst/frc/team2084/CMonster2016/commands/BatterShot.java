/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Sets up for a shot from the batter.
 * 
 * @author Ben Wolsieffer
 */
public class BatterShot extends CommandGroup {

    public BatterShot() {
        addSequential(new SetIntakeCamera(false));
        addParallel(new SetShooterSpeed(ShooterSubsystem.INTAKE_SPEED));
        addSequential(new SetArmAngle(Math.toDegrees(65)));
        addSequential(new SetShooterSpeed(2000));
    }
}
