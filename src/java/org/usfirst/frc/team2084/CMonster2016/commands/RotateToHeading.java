/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.drive.GyroArcadeDriveAlgorithm;

/**
 * Rotates the robot to the specified heading. This command ends when the
 * rotation has been completed. Sometimes, the end condition is not triggered,
 * because PID controllers are hard to tune effectively.
 */
public class RotateToHeading extends ParameterCommand {

    private static final String HEADING_KEY = "Heading";

    private boolean loose = false;
    protected DoubleSupplier headingSupplier;
    private final GyroArcadeDriveAlgorithm arcadeDrive = RobotMap.driveSubsystemArcadeDriveAlgorithm;

    public RotateToHeading(DoubleSupplier heading) {
        headingSupplier = heading;
        init();
    }

    public RotateToHeading(DoubleSupplier heading, double timeout) {
        this(heading, false, timeout);
    }

    public RotateToHeading(DoubleSupplier heading, boolean loose, double timeout) {
        headingSupplier = heading;
        this.loose = loose;
        setTimeout(timeout);
        init();
    }

    public RotateToHeading(double heading, double timeout) {
        this(heading, false, timeout);
    }

    public RotateToHeading(double heading, boolean loose, double timeout) {
        addNumberParameter(HEADING_KEY, heading);
        headingSupplier = () -> getNumberParameter(HEADING_KEY);

        this.loose = loose;
        setTimeout(timeout);

        init();
    }

    private void init() {
        requires(Robot.driveSubsystem);
        arcadeDrive.setTolerance(
                loose ? RobotMap.DRIVE_SUBSYSTEM_LOOSE_HEADING_TOLERANCE : RobotMap.DRIVE_SUBSYSTEM_HEADING_TOLERANCE);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveSubsystem.setEncodersEnabled(true);
        execute();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        arcadeDrive.rotateTo(headingSupplier.getAsDouble());
    }

    /**
     * @return true when the heading is on target, the command times out, or the
     *         navX disconnects
     */
    @Override
    protected boolean isFinished() {
        return arcadeDrive.isHeadingOnTarget() || isTimedOut()
                || /* If we lose the gyro, stop */!RobotMap.driveSubsystemNavX.isConnected();
    }

    /**
     * Stops the drive train.
     */
    @Override
    protected void end() {
        RobotMap.driveSubsystemArcadeDriveAlgorithm.stop();
        arcadeDrive.setTolerance(RobotMap.DRIVE_SUBSYSTEM_HEADING_TOLERANCE);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
