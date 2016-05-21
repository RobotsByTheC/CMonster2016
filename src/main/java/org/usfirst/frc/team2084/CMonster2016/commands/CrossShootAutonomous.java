/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Autonomous mode that drives under the low bar and shoots using the vision
 * system.
 * 
 * @author Ben Wolsieffer
 */
public class CrossShootAutonomous extends CommandGroup {

    public static final double SETTLING_TIME = 0.75;

    public CrossShootAutonomous(RobotMap.AutonomousMode mode) {
        addParallel(new SetCameraAutoExposure(false));
        addSequential(new CrossAutonomous(mode.trajectory));
        // Make sure the ball is out of the shooter wheels
        addParallel(new SetShooterSpeed(-1000));
        addSequential(new SetArmAngle(Math.toRadians(40)));
        addSequential(new WaitCommand(SETTLING_TIME));
        addSequential(new AimAndFire(mode));
        addSequential(new StopShooter());
    }

    /**
     * 
     */
    @Override
    protected void initialize() {
    }
}
