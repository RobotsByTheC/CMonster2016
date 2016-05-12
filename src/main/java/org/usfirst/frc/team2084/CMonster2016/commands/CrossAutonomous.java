/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Autonomous mode that drives over a defense.
 * 
 * @author Ben Wolsieffer
 */
public class CrossAutonomous extends CommandGroup {

    public static final double DRIVE_SPEED = 0.8;

    public CrossAutonomous() {
        this(3.3);
    }
    
    public CrossAutonomous(double time) {
        this(() -> time);
    }

    public CrossAutonomous(DoubleSupplier time) {
        addSequential(new AutonomousSetup());
        addParallel(new SetArmAngle(ArmSubsystem.AIM_ANGLE));
        addParallel(new SequentialCommandGroup(new WaitCommand(0.75), new DriveHeading(() -> 0, () -> DRIVE_SPEED, time)));
    }
}
