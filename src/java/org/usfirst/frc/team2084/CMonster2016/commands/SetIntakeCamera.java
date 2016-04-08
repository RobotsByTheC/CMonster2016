/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.vision.VisionParameters;

/**
 * Sets whether to send the stream from the intake camera or the aiming camera.
 * 
 * @author Ben Wolsieffer
 */
public class SetIntakeCamera extends ParameterCommand {

    public static final String ENABLED_KEY = "Enabled";

    /**
     * Sets whether to send the stream from the intake camera or the aiming
     * camera.
     * 
     * @param enabled if true, stream the intake camera
     */
    public SetIntakeCamera(boolean enabled) {
        setRunWhenDisabled(true);
        addBooleanParameter(ENABLED_KEY, enabled);
    }

    @Override
    protected void initialize() {
        boolean enable = getBooleanParameter(ENABLED_KEY);

        VisionParameters.setIntakeCamera(enable);
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
