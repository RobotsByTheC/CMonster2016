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
 * Command that allows direct control of the arm using a stick on the secondary
 * controller. This should never be needed in competition, as the arm should be
 * entirely under PID control.
 *
 * @author Ben Wolsieffer
 */
public class ManualArmControl extends Command {

    public ManualArmControl() {
        requires(Robot.armSubsystem);
    }

    /**
     * Does nothing.
     */
    @Override
    protected void initialize() {
    }

    /**
     * Updates the arm speed based on the x axis of the controller.
     */
    @Override
    protected void execute() {
        Robot.armSubsystem.setSpeed(Robot.oi.secondaryJoystick.getX());
    }

    /**
     * This command never ends on its own but it could be interrupted.
     *
     * @return false
     */
    @Override
    protected boolean isFinished() {
        return false;
    }

    /**
     * Stops the arm motors.
     */
    @Override
    protected void end() {
        Robot.armSubsystem.stop();
    }

    /**
     * Stops the arm motors.
     */
    @Override
    protected void interrupted() {
        end();
    }
}
