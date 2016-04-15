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

    public CrossShootAutonomous(RobotMap.AutonomousMode mode) {
        addSequential(new CrossAutonomous(mode.time));
        // Get the robot and arm into a position where the camera can see the
        // goal
        addSequential(new RotateToHeading(mode.heading, true, 5));
        // Make sure the ball is out of the shooter wheels
        addParallel(new SetShooterSpeed(-1000));
        addSequential(new WaitCommand(0.5));
        addSequential(new AimAndFire());
        addSequential(new StopShooter());
    }

    /**
     * 
     */
    @Override
    protected void initialize() {
    }
}
