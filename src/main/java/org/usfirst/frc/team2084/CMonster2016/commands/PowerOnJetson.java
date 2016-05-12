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

    @Override
    protected void initialize() {
        RobotMap.shooterSubsystemJetsonPower.set(false);
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
        RobotMap.shooterSubsystemJetsonPower.set(true);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
