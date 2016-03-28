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
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Utility class that generates trajectories in the background.
 * 
 * @author Ben Wolsieffer
 */
public class TrajectoryGenerator {

    private static final ExecutorService pool = Executors.newCachedThreadPool();

    public static Future<TankModifier> generate(Waypoint[] waypoints, Trajectory.Config config) {
        return pool.submit(() -> {
            Trajectory t = Pathfinder.generate(waypoints, config);
            return new TankModifier(t).modify(RobotMap.DRIVE_SUBSYSTEM_WHEELBASE_WIDTH);
        });
    }
}
