/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Homes the arm, lowering it until both sides hit the limit switches. Then it
 * resets the arm angle.
 */
public class HomeArm extends Command {

    public static final double HOMING_SPEED = -1;
    /**
     * In case our crappy limit switches fail, this makes sure that the command
     * still ends and doesn't kill the motors.
     */
    public static final double MAX_TIME = 10;

    private final ArmSubsystem armSubsystem = Robot.armSubsystem;

    public HomeArm() {
        requires(Robot.armSubsystem);

        setTimeout(MAX_TIME);
    }

    @Override
    protected void initialize() {
        // Turn off safety features and the brakes, very dangerous :)
        armSubsystem.setLimitsEnabled(false);
        armSubsystem.setBrakeEnabled(false);
    }

    @Override
    protected void execute() {
        armSubsystem.setSpeed(HOMING_SPEED);

    }

    @Override
    protected boolean isFinished() {
        return (RobotMap.armSubsystemLeftTalon.isRevLimitSwitchClosed()
                && RobotMap.armSubsystemRightTalon.isRevLimitSwitchClosed()) || isTimedOut();
    }

    /**
     * Stops the arm and resets its angle.
     */
    @Override
    protected void end() {
        interrupted();
        Robot.armSubsystem.resetAngle();
    }

    /**
     * Stops the arm, but does not reset its angle.
     */
    @Override
    protected void interrupted() {
        Robot.armSubsystem.stop();
        armSubsystem.setBrakeEnabled(true);
        // armSubsystem.setLimitsEnabled(true);
    }
}
