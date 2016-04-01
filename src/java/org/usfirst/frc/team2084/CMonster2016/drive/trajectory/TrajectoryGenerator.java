/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.drive.trajectory;

import static java.lang.Math.*;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;

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
            Trajectory l = new Trajectory(t.length());
            Trajectory r = new Trajectory(t.length());

            pathfinder_modify_tank(t.segments, l.segments, r.segments, RobotMap.DRIVE_SUBSYSTEM_WHEELBASE_WIDTH);

            return new Trajectory[] { l, r };
        });
    }

    private static void pathfinder_modify_tank(Segment[] original, Segment[] left_traj, Segment[] right_traj,
            double wheelbase_width) {
        double w = wheelbase_width / 2;

        int i;
        for (i = 0; i < original.length; i++) {
            Segment seg = original[i];
            Segment left = seg;
            Segment right = seg;

            double cos_angle = cos(seg.heading);
            double sin_angle = sin(seg.heading);

            left.x = seg.x - (w * sin_angle);
            left.y = seg.y + (w * cos_angle);

            if (i > 0) {
                Segment last = original[i - 1];
                double distance = sqrt((left.x - last.x) * (left.x - last.x) + (left.y - last.y) * (left.y - last.y));

                left.position = last.position + distance;
                left.velocity = distance / seg.dt;
                left.acceleration = (left.velocity - last.velocity) / seg.dt;
                left.jerk = (left.acceleration - last.acceleration) / seg.dt;
            }

            right.x = seg.x + (w * sin_angle);
            right.y = seg.y - (w * cos_angle);

            if (i > 0) {
                Segment last = original[i - 1];
                double distance =
                        sqrt((right.x - last.x) * (right.x - last.x) + (right.y - last.y) * (right.y - last.y));

                right.position = last.position + distance;
                right.velocity = distance / seg.dt;
                right.acceleration = (right.velocity - last.velocity) / seg.dt;
                right.jerk = (right.acceleration - last.acceleration) / seg.dt;
            }

            left_traj[i] = left;
            right_traj[i] = right;
        }
    }
}
