/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.concurrent.Future;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Autonomous mode that drives under the low bar and shoots using the vision
 * system.
 * 
 * @author Ben Wolsieffer
 */
public class CrossShootAutonomous extends CommandGroup {

    public static final double MOVEMENT_WAIT_TIME = 2;

    public static final String ROTATION_KEY = "Auto rotation angle";

    public CrossShootAutonomous(Future<TankModifier> trajectory) {
        // Get the robot and arm into a position where the camera can see the
        // goal
        addSequential(new CrossAutonomous());
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
