/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Sets the speed of the shooter. Since the shooter requires constant updating
 * to stay running, this command never ends on its own. When it gets interrupted
 * or ends for another reason, the shooter coasts to a stop.
 */
@Parameter(key = SetShooterSpeed.DEBUG_KEY, type = Type.BOOLEAN, booleanValue = false)
public class SetShooterSpeed extends ParameterCommand {

    public static final String DEBUG_KEY = "debug";

    private final static String SPEED_KEY = "Speed";

    private double speed;
    private DoubleSupplier speedSupplier;

    public static final String LEFT_VELOCITY_KEY = "left_velocity";
    public static final String RIGHT_VELOCITY_KEY = "right_velocity";

    private static ParameterBundle<SetShooterSpeed> parameters =
            new ParameterBundle<>("Set Shooter Speed", SetShooterSpeed.class);
    private static ITable DEBUG_TABLE = parameters.getTable();

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

    private double[] leftVelocity = new double[3];
    private double[] rightVelocity = new double[3];

    @Override
    protected void execute() {
        Robot.shooterSubsystem.setShooterSpeed(speed);

        if (parameters.getBoolean(DEBUG_KEY)) {
            double time = Timer.getFPGATimestamp();
            leftVelocity[0] = time;
            rightVelocity[0] = time;

            leftVelocity[1] = speed;
            rightVelocity[1] = speed;

            leftVelocity[2] = Robot.shooterSubsystem.getLeftSpeed();
            rightVelocity[2] = Robot.shooterSubsystem.getRightSpeed();

            DEBUG_TABLE.putNumberArray(LEFT_VELOCITY_KEY, leftVelocity);
            DEBUG_TABLE.putNumberArray(RIGHT_VELOCITY_KEY, rightVelocity);
        }
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
