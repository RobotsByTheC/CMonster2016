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
 * Command that turns off the NVidia Jetson TK1.
 *
 * @author Ben Wolsieffer
 */
public class PowerOffJetson extends Command {

    public PowerOffJetson() {
        setRunWhenDisabled(true);
    }

    @Override
    protected void initialize() {
        VisionParameters.shutdown();
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
