/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.drive.DriveUtils;
import org.usfirst.frc.team2084.CMonster2016.drive.PIDConstants;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Follows the specified trajectory, which is read from a file.
 * 
 * @author Ben Wolsieffer
 */
public class PathFollower extends Command {

    public static final String TRAJECTORY_P_KEY = "Trajectory P";
    public static final String TRAJECTORY_I_KEY = "Trajectory I";
    public static final String TRAJECTORY_D_KEY = "Trajectory D";
    public static final String TRAJECTORY_V_KEY = "Trajectory V";
    public static final String TRAJECTORY_A_KEY = "Trajectory A";
    public static final String TRAJECTORY_TURN_KEY = "Trajectory Turn";

    static {
        PIDConstants pid = RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PID_CONSTANTS;

        SmartDashboard.putNumber(TRAJECTORY_P_KEY, pid.p);
        SmartDashboard.putNumber(TRAJECTORY_I_KEY, pid.i);
        SmartDashboard.putNumber(TRAJECTORY_D_KEY, pid.d);
        SmartDashboard.putNumber(TRAJECTORY_V_KEY, pid.f);
        SmartDashboard.putNumber(TRAJECTORY_A_KEY, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_ACC_F);
        SmartDashboard.putNumber(TRAJECTORY_TURN_KEY, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_TURN);
    }

    private final Notifier trajectoryTimer = new Notifier(new TrajectoryTask());

    private final DistanceFollower leftFollower = new DistanceFollower();
    private final DistanceFollower rightFollower = new DistanceFollower();

    private final Future<TankModifier> trajectory;

    private boolean finished = false;

    private class TrajectoryTask implements Runnable {

        @Override
        public void run() {

            double leftSpeed = leftFollower.calculate((int) (Robot.driveSubsystem.getLeftWheels().getDistance()));
            double rightSpeed = rightFollower.calculate((int) (Robot.driveSubsystem.getRightWheels().getDistance()));

            double goalHeading = leftFollower.getHeading();
            double observedHeading = RobotMap.driveSubsystemArcadeDriveAlgorithm.getHeading();
            double angleDiffRads = DriveUtils.normalizeHeading(observedHeading - goalHeading);

            double turn = SmartDashboard.getNumber(TRAJECTORY_TURN_KEY, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_TURN)
                    * angleDiffRads;

            RobotMap.driveSubsystemDriveController.drive(leftSpeed + turn, rightSpeed - turn);

            finished = leftFollower.isFinished() && rightFollower.isFinished();
        }
    }

    public PathFollower(Future<TankModifier> trajectory) {
        this.trajectory = trajectory;
    }

    @Override
    protected void initialize() {
        try {
            trajectoryTimer.stop();

            TankModifier tMod = trajectory.get();

            leftFollower.setTrajectory(tMod.getLeftTrajectory());
            rightFollower.setTrajectory(tMod.getRightTrajectory());

            PIDConstants pid = RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PID_CONSTANTS;

            double p = SmartDashboard.getNumber(TRAJECTORY_P_KEY, pid.p);
            double i = SmartDashboard.getNumber(TRAJECTORY_I_KEY, pid.i);
            double d = SmartDashboard.getNumber(TRAJECTORY_D_KEY, pid.d);
            double f = SmartDashboard.getNumber(TRAJECTORY_V_KEY, pid.f);
            double accF = SmartDashboard.getNumber(TRAJECTORY_A_KEY, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_ACC_F);

            leftFollower.configurePIDVA(p, i, d, f, accF);
            rightFollower.configurePIDVA(p, i, d, f, accF);

            Robot.driveSubsystem.resetEncoders();

            Robot.driveSubsystem.setEncodersEnabled(false);

            trajectoryTimer.startPeriodic(RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PERIOD);
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
        trajectoryTimer.stop();

    }

    @Override
    protected void interrupted() {
        end();
    }
}
