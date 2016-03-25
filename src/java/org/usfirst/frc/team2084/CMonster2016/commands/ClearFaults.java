/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Clears sticky faults of the PDP and Talon SRXs.
 * 
 * @author Ben Wolsieffer
 */
public class ClearFaults extends Command {

    public ClearFaults() {
        setRunWhenDisabled(true);
    }

    /**
     * Clears the sticky faults.
     */
    @Override
    protected void initialize() {
        Robot.pdp.clearStickyFaults();
        RobotMap.armSubsystemLeftTalon.clearStickyFaults();
        RobotMap.armSubsystemRightTalon.clearStickyFaults();
        RobotMap.shooterSubsystemLeftTalon.clearStickyFaults();
        RobotMap.shooterSubsystemRightTalon.clearStickyFaults();
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
