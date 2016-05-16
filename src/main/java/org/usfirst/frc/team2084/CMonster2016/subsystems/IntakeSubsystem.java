/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.subsystems;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for the intake.
 */
@Parameter(key = IntakeSubsystem.IN_SPEED_KEY, type = Type.NUMBER, numberValue = IntakeSubsystem.DEFAULT_IN_SPEED)
@Parameter(key = IntakeSubsystem.OUT_SPEED_KEY, type = Type.NUMBER, numberValue = IntakeSubsystem.DEFAULT_OUT_SPEED)
public class IntakeSubsystem extends Subsystem {

    public static final String IN_SPEED_KEY = "in_speed";
    public static final String OUT_SPEED_KEY = "out_speed";

    public enum State {
        IN(() -> parameters.getNumber(IN_SPEED_KEY)),
        OUT(() -> parameters.getNumber(OUT_SPEED_KEY)),
        STOP(() -> 0);

        private final DoubleSupplier speed;

        private State(DoubleSupplier speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed.getAsDouble();
        }
    }

    /**
     * Default speed used for intaking a ball.
     */
    public static final double DEFAULT_IN_SPEED = 1;

    /**
     * Default speed used to expel a ball. Not really used unless the ball gets
     * stuck.
     */
    public static final double DEFAULT_OUT_SPEED = -1;

    private static final ParameterBundle<IntakeSubsystem> parameters =
            new ParameterBundle<>("Intake Subsystem", IntakeSubsystem.class);

    private final SpeedController talon = RobotMap.intakeSubsystemSpark;

    public void setState(State state) {
        setSpeed(state.getSpeed());
    }

    /**
     * Sets the power of the intake.
     * 
     * @param speed the intake power, between -1.0 and 1.0
     */
    public void setSpeed(double speed) {
        talon.set(speed);
    }

    @Override
    public void initDefaultCommand() {
    }
}
