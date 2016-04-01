/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.drive.processors.InertiaGenerator;
import org.usfirst.frc.team2084.CMonster2016.drive.processors.RescalingDeadband;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Command that drives in arcade drive mode. This is the drive mode used by our
 * robot.
 *
 * @author Ben Wolsieffer
 */
public class ArcadeDrive extends Command {

    private static final String INERTIA_GAIN_KEY = "Inertia Gain";

    private static final double INERTIA_GAIN = 0;
    private static final double DEADBAND = 0.07;
    private static final double MAX_ROTATION = 0.4;

    private final InertiaGenerator inertiaGenerator = new InertiaGenerator(INERTIA_GAIN);
    private final RescalingDeadband deadband = new RescalingDeadband(DEADBAND);

    static {
        SmartDashboard.putNumber(INERTIA_GAIN_KEY, INERTIA_GAIN);
    }

    public ArcadeDrive() {
        // This command drives, so it requires the drive subsystem.
        requires(Robot.driveSubsystem);
    }

    @Override
    protected void initialize() {
        Robot.driveSubsystem.setEncodersEnabled(true);
    }

    /**
     * Updates the robot speed and rotation based on the values of the drive
     * joystick.
     */
    @Override
    protected void execute() {
        Joystick j = Robot.oi.getDriveJoystick();

        // Process the inputs
        double x = deadband.process(j.getX());
        double y = deadband.process(j.getY());
        x *= x * x * MAX_ROTATION;
        y *= y * (y < 0 ? -1 : 1);

        inertiaGenerator.setInertiaGain(SmartDashboard.getNumber(INERTIA_GAIN_KEY, INERTIA_GAIN));

        SmartDashboard.putNumber("Joystick X", x);
        SmartDashboard.putNumber("Joystick Y", y);
        RobotMap.driveSubsystemArcadeDriveAlgorithm.arcadeDrive(-y, inertiaGenerator.process(x));
    }

    /**
     * This command never ends on its own but it could be interrupted.
     *
     * @return false
     */
    @Override
    protected boolean isFinished() {
        return false;
    }

    /**
     * Stops the drive motors.
     */
    @Override
    protected void end() {
        RobotMap.driveSubsystemArcadeDriveAlgorithm.stop();
    }

    /**
     * Stops the drive motors.
     */
    @Override
    protected void interrupted() {
        end();
    }
}
