/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.URISyntaxException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Timer;
import java.util.TimerTask;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.drive.PIDConstants;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.TrajectoryFollower;
import com.team254.lib.trajectory.io.TextFileDeserializer;
import com.team254.lib.util.ChezyMath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Follows the specified trajectory, which is read from a file. This uses 254's
 * trajectory library.
 * 
 * @author Ben Wolsieffer
 */
public class PathFollower extends ParameterCommand {

    public static final String TRAJECTORY_P_KEY = "Trajectory P";
    public static final String TRAJECTORY_I_KEY = "Trajectory I";
    public static final String TRAJECTORY_D_KEY = "Trajectory D";
    public static final String TRAJECTORY_F_KEY = "Trajectory F";
    public static final String TRAJECTORY_A_KEY = "Trajectory A";
    public static final String TRAJECTORY_TURN_KEY = "Trajectory Turn";

    static {
        PIDConstants pid = RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PID_CONSTANTS;

        SmartDashboard.putNumber(TRAJECTORY_P_KEY, pid.p);
        SmartDashboard.putNumber(TRAJECTORY_I_KEY, pid.i);
        SmartDashboard.putNumber(TRAJECTORY_D_KEY, pid.d);
        SmartDashboard.putNumber(TRAJECTORY_F_KEY, pid.f);
        SmartDashboard.putNumber(TRAJECTORY_A_KEY, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_ACC_F);
        SmartDashboard.putNumber(TRAJECTORY_TURN_KEY, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_TURN);
    }

    private static final String NAME_KEY = "Name";

    private volatile Path path;

    private final Timer trajectoryTimer = new Timer(true);

    private final TrajectoryFollower leftFollower = new TrajectoryFollower();
    private final TrajectoryFollower rightFollower = new TrajectoryFollower();

    private boolean finished = false;

    private class TrajectoryTask extends TimerTask {

        @Override
        public void run() {

            double leftSpeed = leftFollower.calculate(Robot.driveSubsystem.getLeftWheels().getDistance());
            double rightSpeed = rightFollower.calculate(Robot.driveSubsystem.getRightWheels().getDistance());

            double goalHeading = leftFollower.getHeading();
            double observedHeading = RobotMap.driveSubsystemArcadeDriveAlgorithm.getHeading();
            double angleDiffRads = ChezyMath.getDifferenceInAngleRadians(observedHeading, goalHeading);
            double angleDiff = Math.toDegrees(angleDiffRads);

            double turn =
                    SmartDashboard.getNumber(TRAJECTORY_TURN_KEY, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_TURN) * angleDiff;

            RobotMap.driveSubsystemDriveController.drive(leftSpeed + turn, rightSpeed - turn);

            finished = leftFollower.isFinishedTrajectory() && rightFollower.isFinishedTrajectory();

            if (finished) {
                cancel();
            }
        }
    }

    public PathFollower(String pathName) {
        addStringParameter(NAME_KEY, pathName);

        addParameterListener(new ParameterListener() {

            @Override
            public void stringChanged(String name, String pathName) {
                if (name.equals(NAME_KEY)) {
                    String serialized;
                    try {
                        serialized = readPathFile(pathName);
                        path = (new TextFileDeserializer()).deserialize(serialized);

                        leftFollower.setTrajectory(path.getLeftWheelTrajectory());
                        rightFollower.setTrajectory(path.getRightWheelTrajectory());
                    } catch (IOException e) {
                        System.err.println(e);
                    }
                }
            }
        });

    }

    @Override
    protected void initialize() {
        trajectoryTimer.cancel();

        PIDConstants pid = RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PID_CONSTANTS;

        double p = SmartDashboard.getNumber(TRAJECTORY_P_KEY, pid.p);
        double i = SmartDashboard.getNumber(TRAJECTORY_I_KEY, pid.i);
        double d = SmartDashboard.getNumber(TRAJECTORY_D_KEY, pid.d);
        double f = SmartDashboard.getNumber(TRAJECTORY_F_KEY, pid.f);
        double accF = SmartDashboard.getNumber(TRAJECTORY_A_KEY, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_ACC_F);

        leftFollower.configure(p, i, d, f, accF);
        rightFollower.configure(p, i, d, f, accF);

        Robot.driveSubsystem.setEncodersEnabled(false);

        trajectoryTimer.scheduleAtFixedRate(new TrajectoryTask(), 0, 10);
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
        trajectoryTimer.cancel();

    }

    @Override
    protected void interrupted() {
        end();
    }

    private static String readPathFile(String name) throws IOException {
        System.out.println("Loading robot path: " + name);

        java.nio.file.Path externalPath = Paths.get("paths/" + name + ".txt");
        java.nio.file.Path internalPath = null;

        java.nio.file.Path foundPath = null;

        try {
            internalPath = Paths.get(PathFollower.class.getResource("/path/" + name + ".txt").toURI());
        } catch (URISyntaxException e) {
        }

        if (Files.exists(externalPath)) {
            foundPath = externalPath;
        } else if (internalPath != null && Files.exists(internalPath)) {
            foundPath = internalPath;
        }

        if (foundPath != null) {
            byte[] encoded = Files.readAllBytes(externalPath);
            return new String(encoded, Charset.defaultCharset());
        } else {
            throw new FileNotFoundException("Could not find file for path: " + name);
        }

    }
}
