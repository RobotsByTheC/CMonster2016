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

import edu.wpi.first.wpilibj.command.Command;

/**
 * Follows the specified trajectory, which is read from a file. This uses 254's
 * trajectory library.
 * 
 * @author Ben Wolsieffer
 */
public class PathFollower extends Command {

    private Path path;

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

            double turn = RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_TURN * angleDiff;

            RobotMap.driveSubsystemDriveController.drive(leftSpeed + turn, rightSpeed - turn);

            finished = leftFollower.isFinishedTrajectory() && rightFollower.isFinishedTrajectory();

            if (finished) {
                cancel();
            }
        }
    }

    public PathFollower(String pathName) {
        String serialized;
        try {
            serialized = readPathFile(pathName);
            path = (new TextFileDeserializer()).deserialize(serialized);

            leftFollower.setTrajectory(path.getLeftWheelTrajectory());
            rightFollower.setTrajectory(path.getRightWheelTrajectory());
        } catch (IOException e) {
            System.err.println("Could not load path: " + pathName + ", " + e);
        }
    }

    @Override
    protected void initialize() {
        trajectoryTimer.cancel();

        PIDConstants pid = RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_PID_CONSTANTS;

        leftFollower.configure(pid.p, pid.i, pid.d, pid.f, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_ACC_F);
        rightFollower.configure(pid.p, pid.i, pid.d, pid.f, RobotMap.DRIVE_SUBSYSTEM_TRAJECTORY_ACC_F);

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
