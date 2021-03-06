/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Sets the shooter firing servo. Even though the firing servo is part of the
 * shooter, this command does not require the {@link ShooterSubsystem}, because
 * it pretty much separate from it.
 */
public class SetFiringServo extends ParameterCommand {

    public static final String FIRING_SERVO_ACTUATION_KEY = "Firing Servo Actuations";

    public static final String FIRE_KEY = "Fire";

    private boolean m_fire;

    private static int actuations = 0;
    private static boolean lastPosition = false;

    static {
        SmartDashboard.putNumber(FIRING_SERVO_ACTUATION_KEY, actuations);
    }

    /**
     * Sets the firing servo position.
     * 
     * @param fire if true, extend the servo, otherwise retract it
     */
    public SetFiringServo(boolean fire) {
        m_fire = fire;

        addBooleanParameter(FIRE_KEY, fire);
    }

    @Override
    protected void initialize() {
        m_fire = getBooleanParameter(FIRE_KEY);
        if (m_fire != lastPosition) {
            // Send the number of actuations to the SmartDashboard. This was
            // used to see if we could switch to pneumatics without a
            // compressor.
            SmartDashboard.putNumber(FIRING_SERVO_ACTUATION_KEY, ++actuations);
            lastPosition = m_fire;
        }
    }

    @Override
    protected void execute() {
        Robot.shooterSubsystem.setFiringServo(m_fire);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
