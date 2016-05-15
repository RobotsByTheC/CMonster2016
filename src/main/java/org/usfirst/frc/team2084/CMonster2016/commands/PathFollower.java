/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.drive.DriveUtils;
import org.usfirst.frc.team2084.CMonster2016.drive.EncoderWheelController;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;
import org.usfirst.frc.team2084.CMonster2016.util.NetworkTablesLargeArrays;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.tables.ITable;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;

/**
 * Follows the specified trajectory, which is read from a file.
 * 
 * @author Ben Wolsieffer
 */
@Parameter(key = PathFollower.P_KEY, type = Type.NUMBER, numberValue = PathFollower.DEFAULT_P)
@Parameter(key = PathFollower.I_KEY, type = Type.NUMBER, numberValue = PathFollower.DEFAULT_I)
@Parameter(key = PathFollower.D_KEY, type = Type.NUMBER, numberValue = PathFollower.DEFAULT_D)
@Parameter(key = PathFollower.V_KEY, type = Type.NUMBER, numberValue = 1 / RobotMap.DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED)
@Parameter(key = PathFollower.A_KEY, type = Type.NUMBER, numberValue = PathFollower.DEFAULT_A)
@Parameter(key = PathFollower.TURN_KEY, type = Type.NUMBER, numberValue = PathFollower.DEFAULT_TURN)
@Parameter(key = PathFollower.DEBUG_KEY, type = Type.BOOLEAN, booleanValue = false)
public class PathFollower extends ParameterCommand {

    public static final String P_KEY = "p";
    public static final String I_KEY = "i";
    public static final String D_KEY = "d";
    public static final String V_KEY = "v";
    public static final String A_KEY = "a";
    public static final String TURN_KEY = "turn";
    public static final String DEBUG_KEY = "debug";

    public static final double DEFAULT_P = 0.05;
    public static final double DEFAULT_I = 0.0;
    public static final double DEFAULT_D = 0.0;
    public static final double DEFAULT_A = 0.17;
    public static final double DEFAULT_TURN = 0;

    private final Timer trajectoryTimer = new Timer();

    private final DistanceFollower leftFollower = new DistanceFollower();
    private final DistanceFollower rightFollower = new DistanceFollower();

    private final Future<Trajectory[]> trajectory;

    private static final ParameterBundle<PathFollower> parameters =
            new ParameterBundle<>("Path Follower", PathFollower.class);
    private static final ITable parameterTable = parameters.getTable();

    private volatile boolean debug;
    private volatile double trajectoryTurn;

    private volatile boolean finished = false;

    private class TrajectoryTask extends TimerTask {

        private final double[] debuggingValues = new double[6];
        private double i = 0;

        @Override
        public void run() {
            EncoderWheelController<SpeedController> leftWheels = Robot.driveSubsystem.getLeftWheels();
            double leftDistance = leftWheels.getDistance();
            double leftSpeed = leftFollower.calculate(leftDistance);
            EncoderWheelController<SpeedController> rightWheels = Robot.driveSubsystem.getRightWheels();
            double rightDistance = rightWheels.getDistance();
            double rightSpeed = rightFollower.calculate(rightDistance);

            double goalHeading = -leftFollower.getHeading();
            double observedHeading = RobotMap.driveSubsystemArcadeDriveAlgorithm.getHeading();
            double angleDiffRads = DriveUtils.normalizeHeading(observedHeading - goalHeading);

            double turn = trajectoryTurn * angleDiffRads;

            RobotMap.driveSubsystemDriveController.drive(leftSpeed + turn, rightSpeed - turn);

            finished = leftFollower.isFinished();
            if (finished) {
                i = 0;
                cancel();
                RobotMap.driveSubsystemDriveController.stop();
            } else {
                if (debug) {
                    debuggingValues[0] = RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PERIOD * i;
                    System.out.println(i);
                    debuggingValues[1] = -Math.toDegrees(DriveUtils.normalizeHeading(observedHeading));
                    debuggingValues[2] = leftDistance;
                    debuggingValues[3] = rightDistance;
                    debuggingValues[4] = leftWheels.getSpeed();
                    debuggingValues[5] = rightWheels.getSpeed();
                    ++i;
                    parameterTable.putNumberArray("debug_values", debuggingValues);
                }
            }
        }
    }

    public PathFollower(Future<Trajectory[]> trajectory) {
        this.trajectory = trajectory;

        parameters.addListener((key, type, val) -> {
            switch (key) {
            case P_KEY:
            case I_KEY:
            case D_KEY:
            case V_KEY:
            case A_KEY:
                double p = parameters.getNumber(P_KEY);
                double i = parameters.getNumber(I_KEY);
                double d = parameters.getNumber(D_KEY);
                double v = parameters.getNumber(V_KEY);
                double a = parameters.getNumber(A_KEY);
                leftFollower.configurePIDVA(p, i, d, v, a);
                rightFollower.configurePIDVA(p, i, d, v, a);
            break;
            // Must be handled in listener because ParameterBundle is not thread
            // safe
            case TURN_KEY:
                trajectoryTurn = (Double) val;
            break;
            }
        });
    }

    @Override
    protected void initialize() {
        try {
            debug = parameters.getBoolean(DEBUG_KEY);

            finished = false;

            Trajectory[] tMod = trajectory.get();
            Trajectory leftTrajectory = tMod[0];
            Trajectory rightTrajectory = tMod[1];

            leftFollower.reset();
            rightFollower.reset();
            leftFollower.setTrajectory(leftTrajectory);
            rightFollower.setTrajectory(rightTrajectory);

            if (debug) {
                int length = leftTrajectory.length();
                double[] headings = new double[length];
                double[] leftPositions = new double[length];
                double[] rightPositions = new double[length];
                double[] leftVelocities = new double[length];
                double[] rightVelocities = new double[length];
                double[] times = new double[length];
                for (int i = 0; i < length; ++i) {
                    headings[i] = Math.toDegrees(DriveUtils.normalizeHeading(leftTrajectory.segments[i].heading));
                    leftPositions[i] = leftTrajectory.segments[i].position;
                    rightPositions[i] = rightTrajectory.segments[i].position;
                    leftVelocities[i] = leftTrajectory.segments[i].velocity;
                    rightVelocities[i] = rightTrajectory.segments[i].velocity;
                    times[i] = i * RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PERIOD;
                }
                NetworkTablesLargeArrays.putNumberArray(parameterTable, "times", times);
                NetworkTablesLargeArrays.putNumberArray(parameterTable, "headings", headings);
                NetworkTablesLargeArrays.putNumberArray(parameterTable, "left_positions", leftPositions);
                NetworkTablesLargeArrays.putNumberArray(parameterTable, "right_positions", rightPositions);
                NetworkTablesLargeArrays.putNumberArray(parameterTable, "left_velocities", leftVelocities);
                NetworkTablesLargeArrays.putNumberArray(parameterTable, "right_velocities", rightVelocities);
            }

            Robot.driveSubsystem.resetEncoders();

            Robot.driveSubsystem.setEncodersEnabled(false);

            trajectoryTimer.scheduleAtFixedRate(new TrajectoryTask(), 0,
                    (long) (RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PERIOD * 1000));
        } catch (InterruptedException | ExecutionException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
        end();
    }
}
