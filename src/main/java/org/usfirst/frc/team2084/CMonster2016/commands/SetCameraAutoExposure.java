/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;
import org.usfirst.frc.team2084.CMonster2016.vision.VisionParameters;

/**
 * Enables or disables the autoexposure function of the camera.
 * 
 * @author Ben Wolsieffer
 */
@Parameter(key = SetCameraAutoExposure.ENABLE_TIME_KEY, type = Type.NUMBER,
        numberValue = SetCameraAutoExposure.DEFAULT_ENABLE_TIME)
public class SetCameraAutoExposure extends ParameterCommand {

    public static final String ENABLE_TIME_KEY = "enable_time";

    public static final String ENABLED_KEY = "Enabled";

    public static final double DEFAULT_ENABLE_TIME = 0.75;

    private static final ParameterBundle<SetCameraAutoExposure> parameters =
            new ParameterBundle<>("Set Camera Auto Exposure", SetCameraAutoExposure.class);

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
            setTimeout(parameters.getNumber(ENABLE_TIME_KEY));
        } else {
            setTimeout(0);
        }

        VisionParameters.setAimingAutoExposure(enable);
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
