/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.subsystems;

import org.usfirst.frc.team2084.CMonster2015.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem for the container hook arm.
 */
public class ContainerHookSubsystem extends Subsystem {

    public static final double WATCHDOG_TIMEOUT = 0.1;

    public static final double RAISE_SPEED = 0.5;
    public static final double LOWER_SPEED = -0.5;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    DigitalInput upperLimitSwitch = RobotMap.containerHookSubsystemUpperLimitSwitch;
    DigitalInput lowerLimitSwitch = RobotMap.containerHookSubsystemLowerLimitSwitch;
    SpeedController jaguar = RobotMap.containerHookSubsystemJaguar;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private final MotorSafetyHelper watchdog;

    public ContainerHookSubsystem() {
        watchdog = new MotorSafetyHelper((MotorSafety) jaguar);
    }

    public void raiseHook() {
        watchdog.setSafetyEnabled(true);
        jaguar.set(RAISE_SPEED);
    }

    public void lowerHook() {
        watchdog.setSafetyEnabled(true);
        jaguar.set(LOWER_SPEED);
    }

    public void stop() {
        watchdog.setSafetyEnabled(false);
        jaguar.set(0);
    }

    public boolean isHookRaised() {
        return upperLimitSwitch.get();
    }

    public boolean isHookLowered() {
        return lowerLimitSwitch.get();
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
