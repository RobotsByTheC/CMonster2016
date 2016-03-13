/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.Robot;

/**
 * Sets the angle of the arm to the specified value. The parameter on the
 * SmartDashboard is specified in degrees because I don't think in radians.
 * 
 * @author Ben Wolsieffer
 */
public class SetArmAngle extends ParameterCommand {

    private static final String ANGLE_KEY = "Angle";

    private DoubleSupplier angleSupplier;
    private double angle;

    /**
     * Functional constructor that allows the angle to be calculated when the
     * command starts.
     * 
     * @param angle the angle supplier
     */
    public SetArmAngle(DoubleSupplier angle) {
        this.angleSupplier = angle;
        init();
    }

    /**
     * Constant angle constructor.
     * 
     * @param angle the angle in radians
     */
    public SetArmAngle(double angle) {
        addNumberParameter(ANGLE_KEY, Math.toDegrees(angle));

        this.angleSupplier = () -> Math.toRadians(getNumberParameter(ANGLE_KEY));
        init();
    }

    private void init() {
        requires(Robot.armSubsystem);
    }

    @Override
    protected void initialize() {
        angle = angleSupplier.getAsDouble();

        Robot.armSubsystem.setAngle(angle);
        Robot.armSubsystem.resetAverageError();
    }

    @Override
    protected void execute() {
        Robot.armSubsystem.setAngle(angle);
    }

    /**
     * @return true when the arm is on target.
     */
    @Override
    protected boolean isFinished() {
        return Robot.armSubsystem.onTarget();
    }

    @Override
    protected void end() {
        interrupted();
        if (Robot.armSubsystem.onTarget()) {
            Robot.armSubsystem.saveArmAngle();
        }
    }

    @Override
    protected void interrupted() {
        Robot.armSubsystem.stop();
    }
}
