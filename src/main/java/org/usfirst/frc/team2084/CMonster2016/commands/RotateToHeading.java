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
import org.usfirst.frc.team2084.CMonster2016.subsystems.DriveSubsystem.HeadingTolerance;

/**
 * Rotates the robot to the specified heading. This command ends when the
 * rotation has been completed. It allows for two different possible tolerances.
 */
public class RotateToHeading extends ParameterCommand {

    private static final String HEADING_KEY = "Heading";

    private HeadingTolerance tolerance = HeadingTolerance.AIMING;
    protected DoubleSupplier headingSupplier;
    private final GyroArcadeDriveAlgorithm arcadeDrive = RobotMap.driveSubsystemArcadeDriveAlgorithm;

    public RotateToHeading(DoubleSupplier heading) {
        headingSupplier = heading;
        init();
    }

    public RotateToHeading(DoubleSupplier heading, double timeout) {
        this(heading, HeadingTolerance.AIMING, timeout);
    }

    public RotateToHeading(DoubleSupplier heading, HeadingTolerance tolerance, double timeout) {
        headingSupplier = heading;
        this.tolerance = tolerance;
        setTimeout(timeout);
        init();
    }

    public RotateToHeading(double heading, double timeout) {
        this(heading, HeadingTolerance.AIMING, timeout);
    }

    public RotateToHeading(double heading, HeadingTolerance tolerance, double timeout) {
        addNumberParameter(HEADING_KEY, Math.toDegrees(heading));
        headingSupplier = () -> Math.toRadians(getNumberParameter(HEADING_KEY));

        this.tolerance = tolerance;
        setTimeout(timeout);

        init();
    }

    private void init() {
        requires(Robot.driveSubsystem);
        Robot.driveSubsystem.setHeadingTolerance(tolerance);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.driveSubsystem.setEncodersEnabled(true);
        execute();
        arcadeDrive.resetPID();
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
        return arcadeDrive.isHeadingOnTarget() || isTimedOut() || !RobotMap.driveSubsystemNavX.isConnected();
    }

    /**
     * Stops the drive train.
     */
    @Override
    protected void end() {
        RobotMap.driveSubsystemArcadeDriveAlgorithm.stop();
        Robot.driveSubsystem.setHeadingTolerance(HeadingTolerance.AIMING);
    }

    @Override
    protected void interrupted() {
        end();
    }
}
