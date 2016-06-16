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
 * Stops the shooter. This just lets the wheels coast to a stop, rather than
 * using motor power or braking to slow them down.
 * 
 * @author Ben Wolsieffer
 */
public class StopShooter extends Command {

    public StopShooter() {
        requires(Robot.shooterSubsystem);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.shooterSubsystem.stop();
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
        end();
    }
}
