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
 * Sets the speed of the shooter. Since the shooter requires constant updating
 * to stay running, this command never ends on its own. When it gets interrupted
 * or ends for another reason, the shooter coasts to a stop.
 */
public class SetShooterSpeed extends ParameterCommand {

    private final static String SPEED_KEY = "Speed";

    private double speed;
    private DoubleSupplier speedSupplier;

    public SetShooterSpeed(DoubleSupplier speed) {
        speedSupplier = speed;
        init();
    }

    public SetShooterSpeed(double speed) {

        speedSupplier = () -> getNumberParameter(SPEED_KEY);
        addNumberParameter(SPEED_KEY, speed);

        init();
    }

    private void init() {
        requires(Robot.shooterSubsystem);
    }

    @Override
    protected void initialize() {
        Robot.shooterSubsystem.resetAverageError();
        speed = speedSupplier.getAsDouble();
    }

    @Override
    protected void execute() {
        Robot.shooterSubsystem.setShooterSpeed(speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.shooterSubsystem.stop();
    }

    @Override
    protected void interrupted() {
        end();
    }
}
