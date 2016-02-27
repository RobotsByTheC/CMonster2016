/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command that resets the arm angle to zero.
 *
 * @author Ben Wolsieffer
 */
public class ResetArmAngle extends Command {

    public ResetArmAngle() {
        setRunWhenDisabled(true);
    }

    /**
     * Resets the gyro.
     */
    @Override
    protected void initialize() {
        Robot.armSubsystem.resetAngle();
    }

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

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
