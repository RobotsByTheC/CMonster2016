/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.drive.trajectory;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Utility class that generates trajectories in the background.
 * 
 * @author Ben Wolsieffer
 */
public class TrajectoryGenerator {

    private static final ExecutorService pool = Executors.newCachedThreadPool();

    public static Future<Trajectory[]> generate(Waypoint[] waypoints, Trajectory.Config config) {
        return pool.submit(() -> {
            Trajectory t = Pathfinder.generate(waypoints, config);

            TankModifier modifier = new TankModifier(t);
            modifier.modify(RobotMap.DRIVE_SUBSYSTEM_WHEELBASE_WIDTH);
            return new Trajectory[] { modifier.getLeftTrajectory(), modifier.getRightTrajectory() };
        });
    }

    public static Trajectory[] generateRotation(double heading, double initialHeading, Trajectory.Config config) {
        // Time to reach the maximum velocity (if not restricted)
        double accelTime = config.max_velocity / config.max_acceleration;
        // Distance to reach maximum velocity (if not restricted)
        double accelDist = 0.5 * config.max_acceleration * accelTime * accelTime;

        // Time spend at maximum velocity
        double constVelTime;

        // Rotation radius
        double r = RobotMap.DRIVE_SUBSYSTEM_WHEELBASE_WIDTH / 2;

        // Angle to rotate
        double rotation = heading - initialHeading;
        // The absolute distance each wheel will move
        double totalDist = r * Math.abs(rotation);

        // Check to see if robot will have time to reach its maximum velocity
        if (totalDist < accelDist * 2) {
            // If so, reduce the acceleration times
            accelTime = Math.sqrt(2 * (totalDist / config.max_acceleration));
            // The robot is constantly accelerating (or decelerating)
            accelDist = totalDist / 2;
            // The robot is never at constant velocity
            constVelTime = 0;
        } else {
            // Calculate the time the robot will remain at its maximum velocity
            constVelTime = (totalDist - accelDist * 2) * config.max_velocity;
        }

        // Calculate the actual maximum velocity
        double maxVel = config.max_acceleration * accelTime;

        // Total time for the movement
        double totalTime = 2 * accelTime + constVelTime;
        // Number of segments
        int segLength = (int) (totalTime / config.dt);

        Segment[] left = new Segment[segLength];
        Segment[] right = new Segment[segLength];

        int i = 0;
        double t = 0;

        // Generate the segments for the acceleration
        for (i = 0; i <= accelTime / config.dt; ++i) {
            t = i * config.dt;
            right[i] = createSegment(config, t, Math.copySign(config.max_acceleration, rotation), 0, initialHeading);
        }

        // Calculate the angular acceleration from linear
        double angA = Math.copySign(config.max_acceleration / r, rotation);
        // Calculate the change in heading during the acceleration
        initialHeading += 0.5 * angA * accelTime * accelTime;

        // Generate segments when velocity is constant
        for (; i <= (accelTime + constVelTime) / config.dt; ++i) {
            t = i * config.dt;
            right[i] = createSegment(config, t, 0, Math.copySign(maxVel, rotation), initialHeading);
        }

        // Calculate angular constant velocity
        double angV = Math.copySign(maxVel / r, rotation);
        // Calculate new heading
        initialHeading += angV * constVelTime;

        for (; i <= (totalTime - accelTime) / config.dt; ++i) {
            t = i * config.dt;
            right[i] = createSegment(config, t, Math.copySign(config.max_acceleration, -rotation),
                    Math.copySign(maxVel, rotation), initialHeading);
        }

        for (i = 0; i < segLength; i++) {
            Segment l = left[i];
            left[i] = new Segment(l.dt, -l.x, -l.y, -l.position, -l.velocity, -l.acceleration, 0, l.heading);
        }

        return new Trajectory[] { new Trajectory(left), new Trajectory(right) };
    }

    private static Segment createSegment(Trajectory.Config config, double t, double a, double v0, double h0) {
        double r = RobotMap.DRIVE_SUBSYSTEM_WHEELBASE_WIDTH / 2;
        double dist = 0.5 * a * t * t + v0 * t;
        double angle = dist / r;
        double x = Math.sin(angle) * r;
        double y = Math.cos(angle) * r;
        double v = a * t + v0;
        return new Segment(config.dt, x, y, dist, v, a, 0, angle + h0);
    }
}
