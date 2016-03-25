/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;

/**
 * Sets the speed of the intake.
 */
public class SetIntakeSpeed extends ParameterCommand {

    private static final String SPEED_KEY = "Speed";

    private double m_speed;

    /**
     * Sets the intake speed.
     * 
     * @param speed the speed (between -1.0 and 1.0)
     */
    public SetIntakeSpeed(double speed) {
        m_speed = speed;

        requires(Robot.intakeSubsystem);

        addNumberParameter(SPEED_KEY, m_speed);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        m_speed = getNumberParameter(SPEED_KEY);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.intakeSubsystem.setSpeed(m_speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.intakeSubsystem.setSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
