/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.vision.VisionParameters;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Notifies the driver about whether the robot is ready to take a shot. This
 * prints a value on the SmartDashboard.
 * 
 * @author Ben Wolsieffer
 */
public class SetCameraAutoExposure extends Command {

    private static final double TIMEOUT = 0.3;

    private final boolean enabled;

    /**
     * Updates the SmartDashboard indicator based on whether the robot is ready
     * to shoot
     * 
     * @param ready whether the robot is ready
     */
    public SetCameraAutoExposure(boolean enabled) {
        this.enabled = enabled;
        setTimeout(TIMEOUT);
    }

    @Override
    protected void initialize() {
        VisionParameters.setAutoExposure(enabled);
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
        end();
    }
}
