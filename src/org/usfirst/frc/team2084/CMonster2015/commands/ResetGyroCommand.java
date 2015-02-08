/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command that resets the gyro value to zero.
 *
 * @author Ben Wolsieffer
 */
public class ResetGyroCommand extends Command {

    public ResetGyroCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    /**
     * Resets the gyro.
     */
    @Override
    protected void initialize() {
        Robot.driveSubsystem.getMecanumDriveAlgorithm().resetGyro();
    }

    /**
     * Does nothing.
     */
    @Override
    protected void execute() {
    }

    /**
     * This command only needs to run once, so this method always returns true.
     *
     * @return true
     */
    @Override
    protected boolean isFinished() {
        return true;
    }

    /**
     * Does nothing.
     */
    @Override
    protected void end() {
    }

    /**
     * Does nothing.
     */
    @Override
    protected void interrupted() {
    }
}
