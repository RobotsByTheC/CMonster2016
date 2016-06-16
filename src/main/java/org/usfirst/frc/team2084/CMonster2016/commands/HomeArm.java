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
 * backs off and slowly moves forward to precisely home (like a 3D printer).
 * Once homing is complete it resets the arm angle. This used to run at the
 * beginning of every match, but now is only used in the pits, and the arm angle
 * is saved.
 */
public class HomeArm extends Command {

    private enum State {
        HIGH_SPEED,
        REVERSE,
        LOW_SPEED,
        DONE
    }

    private State state = State.HIGH_SPEED;

    public static final double HOMING_HIGH_SPEED = -1;
    public static final double HOMING_REVERSE_SPEED = 0.3;
    public static final double HOMING_LOW_SPEED = -0.2;

    /**
     * In case our limit switches fail, this makes sure that the command still
     * ends and doesn't kill the motors.
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

        state = State.HIGH_SPEED;
    }

    @Override
    protected void execute() {
        boolean left = RobotMap.armSubsystemLeftTalon.isRevLimitSwitchClosed();
        boolean right = RobotMap.armSubsystemRightTalon.isRevLimitSwitchClosed();
        switch (state) {
        case HIGH_SPEED:
            if (!left) {
                armSubsystem.setLeftSpeed(HOMING_HIGH_SPEED);
            }
            if (!right) {
                armSubsystem.setRightSpeed(HOMING_HIGH_SPEED);
            }
            if (left && right) {
                state = State.REVERSE;
            }
        break;
        case REVERSE:
            if (left) {
                armSubsystem.setLeftSpeed(HOMING_REVERSE_SPEED);
            }
            if (right) {
                armSubsystem.setRightSpeed(HOMING_REVERSE_SPEED);
            }
            if (!left && !right) {
                state = State.LOW_SPEED;
            }
        break;
        case LOW_SPEED:
            if (!left) {
                armSubsystem.setLeftSpeed(HOMING_LOW_SPEED);
            }
            if (!right) {
                armSubsystem.setRightSpeed(HOMING_LOW_SPEED);
            }
            if (left && right) {
                state = State.DONE;
            }
        break;
        case DONE:
            armSubsystem.stop();
        }

    }

    @Override
    protected boolean isFinished() {
        return state == State.DONE || isTimedOut();
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
