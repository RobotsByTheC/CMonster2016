/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.subsystems.ShooterSubsystem;
import org.usfirst.frc.team2084.CMonster2016.vision.VisionParameters;
import org.usfirst.frc.team2084.CMonster2016.vision.VisionResults;

import edu.wpi.first.wpilibj.command.ConditionalCommandGroup;
import edu.wpi.first.wpilibj.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Aims the robot and fires the shooter. This command takes away control almost
 * entirely from the driver, and is only used during autonomous.
 * 
 * If the vision data is stale, this command immediately ends without doing
 * anything. This makes sure we don't waste our ball during autonomous if we
 * know we are going to miss.
 * 
 * {@link AimShot} is used during teleop.
 * 
 * @author Ben Wolsieffer
 */
public class AimAndFire extends ConditionalCommandGroup {

    public AimAndFire() {
        addSequential(new SetCameraAutoExposure(false));
        addSequential(new TakeSnapshot());
        addParallel(new SetShooterSpeed(() -> ShooterSubsystem.getCalibrationSpeed(VisionResults.getGoalDistance())));
        // Aim the robot and the arm, but make sure that it takes at least a
        // second to allow the shooter to spin up
        addSequential(new ParallelCommandGroup(new AimArm(), new AimRobot(), new WaitCommand(1)));
        addSequential(new SetFiringServo(true));
    }

    @Override
    protected boolean shouldRun() {
        return !VisionResults.isStale();
    }

    @Override
    protected void end() {
        VisionParameters.setAutoExposure(true);
    }

    /**
     * 
     */
    @Override
    protected void interrupted() {
        end();
    }
}
