/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.subsystems.IntakeSubsystem;
import org.usfirst.frc.team2084.CMonster2016.subsystems.IntakeSubsystem.State;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Sets the speed of the intake.
 */
public class SetIntakeState extends Command {

    private IntakeSubsystem.State state;

    /**
     * Sets the intake speed.
     * 
     * @param speed the speed (between -1.0 and 1.0)
     */
    public SetIntakeState(IntakeSubsystem.State state) {
        this.state = state;

        requires(Robot.intakeSubsystem);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.intakeSubsystem.setState(state);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.intakeSubsystem.setState(State.STOP);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
