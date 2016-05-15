/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.drive.processors.RescalingDeadband;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Command that drives in arcade drive mode. This is the drive mode used by our
 * robot.
 *
 * @author Ben Wolsieffer
 */
public class TankDrive extends Command {

    private static final double DEADBAND = 0.05;

    private final RescalingDeadband deadband = new RescalingDeadband(DEADBAND);

    public TankDrive() {
        // This command drives, so it requires the drive subsystem.
        requires(Robot.driveSubsystem);
    }

    @Override
    protected void initialize() {
        Robot.driveSubsystem.setEncodersEnabled(true);
        RobotMap.driveSubsystemLeftWheels.reset();
        RobotMap.driveSubsystemRightWheels.reset();
    }

    /**
     * Updates the robot speed and rotation based on the values of the drive
     * joystick.
     */
    @Override
    protected void execute() {
        Joystick left = Robot.oi.getLeftDriveJoystick();
        Joystick right = Robot.oi.getRightDriveJoystick();

        // Process the inputs
        double l = deadband.process(-left.getY());
        double r = deadband.process(-right.getY());
        l *= l * (l < 0 ? -1 : 1);
        r *= r * (r < 0 ? -1 : 1);

        RobotMap.driveSubsystemDriveController.drive(l, r);
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
