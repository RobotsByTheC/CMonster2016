/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.test;

import static org.hamcrest.Matchers.lessThanOrEqualTo;
import static org.junit.Assert.*;

import java.util.Arrays;
import java.util.Collection;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.stream.Collectors;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;
import org.junit.runners.Parameterized.Parameter;
import org.junit.runners.Parameterized.Parameters;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;

import jaci.pathfinder.Trajectory;

/**
 * @author Ben Wolsieffer
 */
@RunWith(Parameterized.class)
public class WaypointTest {

    @Parameter
    public Future<Trajectory[]> trajectory;

    @Test
    public void testGenerate() throws InterruptedException, ExecutionException {
        trajectory.get();
    }

    @Test
    public void testVelocity() throws InterruptedException, ExecutionException {
        for (Trajectory t : trajectory.get()) {
            assertThat("Max velocity must not exceed what physics allows",
                    Arrays.stream(t.segments).mapToDouble((s) -> s.velocity).max().getAsDouble(),
                    lessThanOrEqualTo(RobotMap.DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED));
        }
    }

    @SuppressWarnings("unchecked")
    @Parameters
    public static Collection<Future<Trajectory[]>> trajectories() {
        Class<RobotMap> robotMapClass = RobotMap.class;
        return Arrays.stream(robotMapClass.getFields())
                .filter((f) -> f.getType().equals(Future.class) && f.getName().contains("TRAJECTORY")).map((f) -> {
                    try {
                        return (Future<Trajectory[]>) f.get(null);
                    } catch (Exception e) {
                        fail("Could not get trajectory field value:" + e);
                        return null;
                    }
                }).filter((f) -> f != null).collect(Collectors.toList());
    }

}
