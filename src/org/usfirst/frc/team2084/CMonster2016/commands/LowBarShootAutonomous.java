/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * @author Robot
 */
public class LowBarShootAutonomous extends CommandGroup {

    /**
     * 
     */
    public LowBarShootAutonomous() {
        addSequential(new LowBarAutonomous());
        addSequential(new WaitCommand(0.5));
        addSequential(new RotateToHeading(Math.toRadians(25)));
        addSequential(new SetArmAngle(ArmSubsystem.AIM_ANGLE));
        addParallel(new SetShooterSpeed(-1000));
        addSequential(new WaitCommand(1));
        addSequential(new AimAndFire());
    }
}
