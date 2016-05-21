/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.concurrent.Future;

import org.usfirst.frc.team2084.CMonster2016.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.SequentialCommandGroup;
import jaci.pathfinder.Trajectory;

/**
 * Autonomous mode that drives over a defense.
 * 
 * @author Ben Wolsieffer
 */
public class CrossAutonomous extends CommandGroup {

    public CrossAutonomous(Future<Trajectory[]> trajectory) {
        addSequential(new AutonomousSetup());
        addParallel(new SetArmAngle(ArmSubsystem.AIM_ANGLE));
        addParallel(new SequentialCommandGroup(/* new WaitCommand(0.75), */ new PathFollower(trajectory)));
    }
}
