/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.subsystems.ShooterSubsystem;
import org.usfirst.frc.team2084.CMonster2016.vision.VisionResults;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * @author Ben Wolsieffer
 */
public class AimShot extends CommandGroup {

    public AimShot() {
        addSequential(new ShotReadyNotify(false));
        addParallel(new SetShooterSpeed(() -> ShooterSubsystem.getCalibrationSpeed(VisionResults.getGoalDistance())));
        addSequential(new ParallelCommandGroup(new AimArm(), new AimRobot(), new WaitCommand(0.75)));
        addSequential(new ShotReadyNotify(true));
    }
}
