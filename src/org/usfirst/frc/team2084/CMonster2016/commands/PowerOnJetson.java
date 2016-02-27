/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Command that turns on the NVidia Jetson TK1 using a DIO pin. This never
 * seemed to work right and has been replaced by a physical power button on the
 * Jetson.
 *
 * @author Ben Wolsieffer
 */
public class PowerOnJetson extends Command {

    public PowerOnJetson() {
        setTimeout(0.5);
        setRunWhenDisabled(true);
    }

    /**
     * Does nothing.
     */
    @Override
    protected void initialize() {
        RobotMap.shooterSubsystemJetsonPower.set(false);
    }

    /**
     * Updates the arm speed based on the x axis of the controller.
     */
    @Override
    protected void execute() {
    }

    /**
     * This command never ends on its own but it could be interrupted.
     *
     * @return false
     */
    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

    /**
     * Stops the arm motors.
     */
    @Override
    protected void end() {
        RobotMap.shooterSubsystemJetsonPower.set(true);
    }

    /**
     * Stops the arm motors.
     */
    @Override
    protected void interrupted() {
        end();
    }
}
