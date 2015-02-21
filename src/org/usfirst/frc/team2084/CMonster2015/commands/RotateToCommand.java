/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import org.usfirst.frc.team2084.CMonster2015.Robot;
import org.usfirst.frc.team2084.CMonster2015.RobotMap;
import org.usfirst.frc.team2084.CMonster2015.drive.processors.LinearRamper;
import org.usfirst.frc.team2084.CMonster2015.drive.processors.LinearRamper.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Command that rotates the robot to a certain heading.
 *
 * @author Ben Wolsieffer
 */
public class RotateToCommand extends ParameterCommand {

    public static final String HEADING_KEY = "Heading";
    public static final String DEBUG_KEY = "Debug";
    public static final String HEADING_ERROR_KEY = "Heading Error";

    protected boolean debug = false;

    /**
     * The heading the robot should rotate to.
     */
    protected double heading;
    /**
     * The ramper that is used to prevent too much acceleration.
     */
    protected final LinearRamper rotationRamper = new LinearRamper(1.0, Type.UP);

    protected final double maxRotationSpeed;

    /**
     * Creates a new {@link RotateToCommand} the rotates the robot to a heading
     * specified on the SmartDashboard and times out after 5 seconds.
     */
    public RotateToCommand() {
        this(0);
    }

    /**
     * Creates a new {@link RotateToCommand} the rotates the robot to the
     * specified heading and times out after 5 seconds.
     * 
     * @param heading the heading to rotate to
     */
    public RotateToCommand(double heading) {
        this(0, 1);
    }

    /**
     * Creates a new {@link RotateToCommand} the rotates the robot to the
     * specified heading and times out after 5 seconds;
     * 
     * @param heading the heading to rotate to
     * @param maxRotationSpeed the maximum speed to rotate
     */
    public RotateToCommand(double heading, double maxRotationSpeed) {
        this(heading, maxRotationSpeed, 5);
    }

    /**
     * Creates a new {@link RotateToCommand} the rotates the robot to the
     * specified heading and times out after the specified number of seconds.
     * The timeout is used to prevent the robot from waiting indefinitely if it
     * turn to the target heading.
     * 
     * @param heading the heading to rotate to
     * @param maxRotationSpeed the maximum speed to rotate
     * @param timeout the max time the command can take to complete
     */
    public RotateToCommand(double heading, double maxRotationSpeed, double timeout) {
        this(heading, maxRotationSpeed, timeout, false);
    }

    /**
     * Creates a new {@link RotateToCommand} the rotates the robot to the
     * specified heading and times out after the specified number of seconds.
     * The timeout is used to prevent the robot from waiting indefinitely if it
     * turn to the target heading.
     * 
     * @param heading the heading to rotate to
     * @param maxRotationSpeed the maximum speed to rotate
     * @param timeout the max time the command can take to complete
     * @param debug whether to print the error to the SmartDashboard
     */
    public RotateToCommand(double heading, double maxRotationSpeed, double timeout, boolean debug) {
        super(timeout);
        this.maxRotationSpeed = maxRotationSpeed;

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveSubsystem);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        addNumberParameter(HEADING_KEY, heading);
        addBooleanParameter(DEBUG_KEY, debug);
    }

    /**
     * Resets the ramper and gets parameter values from the SmartDashboard.
     */
    @Override
    protected void initialize() {
        heading = getNumberParameter(HEADING_KEY);
        debug = getBooleanParameter(DEBUG_KEY);

        rotationRamper.reset();
    }

    /**
     * Does nothing because the actual turning is done in the
     * {@link #isFinished()} method.
     */
    @Override
    protected void execute() {
        RobotMap.driveSubsystemMecanumDriveAlgorithm.driveFieldHeadingCartesian(0, 0, heading, rotationRamper.process(maxRotationSpeed));

        if (debug) {
            SmartDashboard.putNumber(HEADING_ERROR_KEY, RobotMap.driveSubsystemMecanumDriveAlgorithm.getHeadingError());
        }
    }

    /**
     * Returns true when the robot has rotated to the specified heading.
     * 
     * @return true when the robot rotates to the target heading
     */
    @Override
    protected boolean isFinished() {
        return RobotMap.driveSubsystemMecanumDriveAlgorithm.isHeadingOnTarget() || isTimedOut();
    }

    /**
     * Stops the robot.
     */
    @Override
    protected void end() {
        RobotMap.driveSubsystemMecanumDriveAlgorithm.stop();
    }

    /**
     * Stops the robot.
     */
    @Override
    protected void interrupted() {
        end();
    }
}