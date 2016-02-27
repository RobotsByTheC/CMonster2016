/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.subsystems;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for the intake.
 */
public class IntakeSubsystem extends Subsystem {

    /**
     * The speed used for intaking a ball.
     */
    public static final double IN_SPEED = 1;

    /**
     * The speed used to expel a ball. Not really used unless the ball gets
     * stuck.
     */
    public static final double OUT_SPEED = -1;

    private final SpeedController talon = RobotMap.intakeSubsystemVictor;

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
