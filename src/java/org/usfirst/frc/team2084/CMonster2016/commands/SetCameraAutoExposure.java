/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.vision.VisionParameters;

/**
 * Notifies the driver about whether the robot is ready to take a shot. This
 * prints a value on the SmartDashboard.
 * 
 * @author Ben Wolsieffer
 */
public class SetCameraAutoExposure extends ParameterCommand {

    public static final String ENABLED_KEY = "Enabled";

    public static final double ENABLE_TIME = 0.5;

    /**
     * Updates the SmartDashboard indicator based on whether the robot is ready
     * to shoot
     * 
     * @param ready whether the robot is ready
     */
    public SetCameraAutoExposure(boolean enabled) {
        setRunWhenDisabled(true);
        addBooleanParameter(ENABLED_KEY, enabled);
    }

    @Override
    protected void initialize() {
        boolean enable = getBooleanParameter(ENABLED_KEY);

        if (!enable) {
            setTimeout(ENABLE_TIME);
        } else {
            setTimeout(0);
        }

        VisionParameters.setAutoExposure(enable);
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
