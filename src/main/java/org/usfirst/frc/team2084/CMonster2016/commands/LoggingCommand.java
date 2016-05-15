/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.vision.VisionResults;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Command that is always running and reports debugging values. It is more
 * important than it sounds, and is actually critical to the vision system.
 * Basically, anything that needs to run in the main loop all the time goes in
 * here.
 *
 * @author Ben Wolsieffer
 */
public class LoggingCommand extends Command {

    /**
     * The length of a match in seconds.
     */
    public static final double MATCH_LENGTH = 150;

    public LoggingCommand() {
        // Make sure this command still reports data when the
        // robot is disabled.
        setRunWhenDisabled(true);
    }

    /**
     * Does nothing.
     */
    @Override
    protected void initialize() {
        // SmartDashboard.putNumber("Heading Tolerance",
        // Math.toDegrees(RobotMap.DRIVE_SUBSYSTEM_HEADING_TOLERANCE));

        // SmartDashboard.putNumber(LowBarShootAutonomous.ROTATION_KEY,
        // Preferences.getInstance().getDouble(LowBarShootAutonomous.ROTATION_KEY,
        // 0));

        SmartDashboard.putNumber(AimRobot.GOAL_HEADING_OFFSET_KEY, Math.toDegrees(AimRobot.GOAL_HEADING_OFFSET));

    }

    /**
     * Prints out debugging data to the SmartDashboard.
     */
    @Override
    protected void execute() {

        // Report gyro values
        SmartDashboard.putNumber("Gyro Angle",
                Math.toDegrees(RobotMap.driveSubsystemArcadeDriveAlgorithm.getHeading()));
        SmartDashboard.putBoolean("navX Connected", RobotMap.driveSubsystemNavX.isConnected());
        SmartDashboard.putBoolean("navX Calibrating", RobotMap.driveSubsystemNavX.isCalibrating());

        SmartDashboard.putBoolean("Valid Vision Data", !VisionResults.isStale());

        // Report remaining match time
        double matchTime = Timer.getMatchTime();
        SmartDashboard.putNumber("Time Remaining", matchTime < 0 ? 0 : matchTime);

        // Report arm angle
        double armAngle = Robot.armSubsystem.getAngle();
        SmartDashboard.putNumber("Arm Angle", Math.toDegrees(armAngle));

        // Report shooter wheel speeds
        SmartDashboard.putNumber("Shooter Left Speed", Robot.shooterSubsystem.getLeftSpeed());
        SmartDashboard.putNumber("Shooter Right Speed", Robot.shooterSubsystem.getRightSpeed());

        // Update the current robot heading for the vision system
        VisionResults.setCurrentHeading(RobotMap.driveSubsystemArcadeDriveAlgorithm.getHeading());
        VisionResults.setArmAngle(armAngle);

        SmartDashboard.putBoolean("Encoders Enabled", Robot.driveSubsystem.getEncodersEnabled());
    }

    /**
     * This command never finishes.
     *
     * @return false
     */
    @Override
    protected boolean isFinished() {
        return false;
    }

    /**
     * Does nothing.
     */
    @Override
    protected void end() {
    }

    /**
     * Does nothing.
     */
    @Override
    protected void interrupted() {
    }
}
