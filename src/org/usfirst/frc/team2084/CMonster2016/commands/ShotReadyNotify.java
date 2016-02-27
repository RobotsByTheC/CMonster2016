/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Notifies the driver about whether the robot is ready to take a shot. This
 * prints a value on the SmartDashboard.
 * 
 * @author Ben Wolsieffer
 */
public class ShotReadyNotify extends Command {

    private final boolean ready;

    /**
     * Updates the SmartDashboard indicator based on whether the robot is ready
     * to shoot
     * 
     * @param ready whether the robot is ready
     */
    public ShotReadyNotify(boolean ready) {
        this.ready = ready;
    }

    @Override
    protected void initialize() {
        SmartDashboard.putBoolean("Ready to fire", ready);
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
        end();
    }
}
