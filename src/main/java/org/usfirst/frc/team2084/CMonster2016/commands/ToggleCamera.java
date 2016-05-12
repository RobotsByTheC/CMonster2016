/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.vision.VisionParameters;

/**
 * Toggles the stream between the two cameras.
 * 
 * @author Ben Wolsieffer
 */
public class ToggleCamera extends ParameterCommand {

    /**
     * Sets whether to send the stream from the intake camera or the aiming
     * camera.
     * 
     * @param enabled if true, stream the intake camera
     */
    public ToggleCamera() {
        setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        VisionParameters.setIntakeCamera(!VisionParameters.isIntakeCamera());
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
