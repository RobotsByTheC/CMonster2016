/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.Robot;

/**
 *
 */
public class SetArmAngle extends ParameterCommand {

    private static final String ANGLE_KEY = "Angle";
    private static final String HOLD_KEY = "Hold";

    private DoubleSupplier angleSupplier;
    private BooleanSupplier holdSupplier;
    private double angle;
    private boolean hold;

    public SetArmAngle(DoubleSupplier angle, BooleanSupplier hold) {
        this.angleSupplier = angle;
        this.holdSupplier = hold;
        init();
    }

    public SetArmAngle(double angle, boolean hold) {
        addNumberParameter(ANGLE_KEY, Math.toDegrees(angle));
        addBooleanParameter(HOLD_KEY, hold);

        this.angleSupplier = () -> Math.toRadians(getNumberParameter(ANGLE_KEY));
        this.holdSupplier = () -> getBooleanParameter(HOLD_KEY);
        init();
    }

    public SetArmAngle(double angle) {
        this(angle, false);
    }

    private void init() {
        requires(Robot.armSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        angle = angleSupplier.getAsDouble();
        hold = holdSupplier.getAsBoolean();

        Robot.armSubsystem.setAngle(angle);
        Robot.armSubsystem.resetAverageError();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.armSubsystem.setAngle(angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return hold ? false : Robot.armSubsystem.onTarget();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.armSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
