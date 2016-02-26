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

    public static final double IN_SPEED = 1;
    public static final double OUT_SPEED = -1;

    private final SpeedController talon = RobotMap.intakeSubsystemVictor;

    public void setSpeed(double speed) {
        talon.set(speed);
    }

    @Override
    public void initDefaultCommand() {
    }
}
