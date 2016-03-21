/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;

/**
 * Enables or disable the velocity feedback control on the wheel encoders.
 * 
 * @author Ben Wolsieffer
 */
public class SetEncodersEnabled extends ParameterCommand {

    public static final String ENABLED_KEY = "Enabled";

    /**
     * Updates the SmartDashboard indicator based on whether the robot is ready
     * to shoot
     * 
     * @param ready whether the robot is ready
     */
    public SetEncodersEnabled(boolean enabled) {
        setRunWhenDisabled(true);
        addBooleanParameter(ENABLED_KEY, enabled);
    }

    @Override
    protected void initialize() {
        Robot.driveSubsystem.setEncodersEnabled(getBooleanParameter(ENABLED_KEY));
    }

    @Override
    protected void execute() {
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
    }
}
