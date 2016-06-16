/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;

/**
 * Sets the measured angle of the arm to the specified value. The parameter on
 * the SmartDashboard is specified in degrees because I don't think in radians.
 * 
 * @author Ben Wolsieffer
 */
public class SetMeasuredArmAngle extends ParameterCommand {

    private static final String ANGLE_KEY = "Angle";

    private double angle;

    /**
     * Constant angle constructor.
     * 
     * @param angle the angle in radians
     */
    public SetMeasuredArmAngle(double angle) {
        addNumberParameter(ANGLE_KEY, Math.toDegrees(angle));

        init();
    }

    private void init() {
        setRunWhenDisabled(true);
        requires(Robot.armSubsystem);
    }

    @Override
    protected void initialize() {
        angle = Math.toRadians(getNumberParameter(ANGLE_KEY));

        Robot.armSubsystem.setMeasuredArmAngle(angle);
    }

    @Override
    protected void execute() {
    }

    /**
     * @return true when the arm is on target.
     */
    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
        interrupted();
    }

    @Override
    protected void interrupted() {
    }
}
