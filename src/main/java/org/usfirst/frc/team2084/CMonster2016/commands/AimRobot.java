/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.concurrent.ExecutionException;
import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.RollingAverage;
import org.usfirst.frc.team2084.CMonster2016.drive.DriveUtils;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;
import org.usfirst.frc.team2084.CMonster2016.vision.VisionResults;

import jaci.pathfinder.Trajectory.Segment;

/**
 * Aims the robot to the heading given by the vision system. If the vision data
 * is stale, it immediately ends without doing anything.
 */
@Parameter(key = AimRobot.MAX_ROTATION_UPDATE_KEY, type = Type.NUMBER,
        numberValue = AimRobot.DEFAULT_MAX_ROTATION_UPDATE)
@Parameter(key = AimRobot.HEADING_OFFSET_KEY, type = Type.NUMBER, numberValue = AimRobot.DEFAULT_HEADING_OFFSET)
public class AimRobot extends RotateToHeading {

    public static final String MAX_ROTATION_UPDATE_KEY = "max_rotation_update";
    public static final String HEADING_OFFSET_KEY = "heading_offset";

    /**
     * Default maximum rotation rate (deg/sec) under which to update the target
     * heading.
     */
    public static final double DEFAULT_MAX_ROTATION_UPDATE = 45;
    /**
     * Default amount to add to the calculated desired heading when aiming
     */
    public static final double DEFAULT_HEADING_OFFSET = 3.15;

    // BAD - must be static to work
    private static final RollingAverage headingAverage = new RollingAverage(40);

    private static final ParameterBundle<AimRobot> parameters = new ParameterBundle<>("Aim Robot", AimRobot.class);

    public static final double DEFAULT_TIMEOUT = 5;
    private boolean stale = false;

    private final RobotMap.AutonomousMode mode;

    public AimRobot() {
        this(null);
    }

    public AimRobot(RobotMap.AutonomousMode mode) {
        super(new DoubleSupplier() {

            private double lastHeading = Double.MAX_VALUE;

            @Override
            public double getAsDouble() {
                double heading = DriveUtils.normalizeHeading(getAimHeading());
                // Only update the heading if the robot is not rotating very
                // fast. This helps compensate for lag in the camera.
                if (Math.abs(RobotMap.driveSubsystemArcadeDriveAlgorithm.getRotationRate()) > Math
                        .toRadians(parameters.getNumber(MAX_ROTATION_UPDATE_KEY))) {
                    heading = lastHeading;
                } else {
                    // System.out.println("update");
                }

                lastHeading = heading;

                headingAverage.newValue(heading);

                return headingAverage.getAverage();
            }
        });
        this.mode = mode;
    }

    private static double getAimHeading() {
        return VisionResults.getGoalHeading() + Math.toRadians(parameters.getNumber(HEADING_OFFSET_KEY));
    }

    @Override
    protected void initialize() {
        super.initialize();
        // If this is used during autonomous, dynamically set the timeout to
        // leave just enough time to fire the ball (because we known exactly how
        // long the trajectory takes)
        if (mode != null) {
            try {
                Segment[] leftSegments = mode.trajectory.get()[0].segments;
                double driveTime = leftSegments[0].dt * leftSegments.length;
                setTimeout(14.25 - CrossShootAutonomous.SETTLING_TIME - driveTime);
            } catch (InterruptedException | ExecutionException e) {
            }
        }
        stale = VisionResults.isStale();
        headingAverage.reset(getAimHeading());
    }

    @Override
    protected boolean isFinished() {
        return super.isFinished() || stale;
    }
}
