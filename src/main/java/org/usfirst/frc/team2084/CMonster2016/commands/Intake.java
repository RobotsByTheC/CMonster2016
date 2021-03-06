/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.subsystems.IntakeSubsystem.State;
import org.usfirst.frc.team2084.CMonster2016.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Intakes the ball.
 * 
 * @author Ben Wolsieffer
 */
public class Intake extends CommandGroup {

    public Intake() {
        addParallel(new SetFiringServo(false));
        addParallel(new SetIntakeState(State.IN));
        addParallel(new SetShooterSpeed(ShooterSubsystem.INTAKE_SPEED));
    }
}
