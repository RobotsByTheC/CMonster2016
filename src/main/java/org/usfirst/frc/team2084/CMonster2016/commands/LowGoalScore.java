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
 * Scores the ball in the low goal.
 * 
 * @author Ben Wolsieffer
 */
public class LowGoalScore extends CommandGroup {

    public LowGoalScore() {
        addParallel(new SetShooterSpeed(ShooterSubsystem.LOW_GOAL_SPEED));
        addParallel(new SetFiringServo(true));
    }
}
