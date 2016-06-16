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

    @Override
    protected void initialize() {
        m_speed = getNumberParameter(SPEED_KEY);
    }

    @Override
    protected void execute() {
        Robot.intakeSubsystem.setSpeed(m_speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.intakeSubsystem.setSpeed(0);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
