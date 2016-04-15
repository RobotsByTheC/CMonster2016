/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.RollingAverage;
import org.usfirst.frc.team2084.CMonster2016.vision.VisionResults;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Aims the robot to the heading given by the vision system. If the vision data
 * is stale, it immediately ends without doing anything.
 */
public class AimRobot extends RotateToHeading {

    public static final double GOAL_HEADING_OFFSET = Math.toRadians(6.3);
    public static final String GOAL_HEADING_OFFSET_KEY = "Goal Heading Offset";

    public static final double HEADING_CHANGE_TOLERANCE = Math.toDegrees(7);

    // BAD
    private static final RollingAverage headingAverage = new RollingAverage(500);

    
    public static final double TIMEOUT = 5;
    private boolean stale = false;

    public AimRobot(boolean shouldTimeout) {
        super(new DoubleSupplier() {
            

            private double lastHeading = Double.MAX_VALUE;

            @Override
            public double getAsDouble() {
                double heading = getAimHeading();
                // Only update the heading if it has not changed by a huge
                // amount
                if (lastHeading < Double.MAX_VALUE && Math.abs(heading - lastHeading) > HEADING_CHANGE_TOLERANCE) {
                    heading = lastHeading;
                }

                lastHeading = heading;

                headingAverage.newValue(heading);

                return headingAverage.getAverage();
            }
        });
        if (shouldTimeout) {
            setTimeout(TIMEOUT);
        }
    }

    public AimRobot() {
        this(true);
    }

    private static double getAimHeading() {
        return VisionResults.getGoalHeading() + Math.toRadians(SmartDashboard.getNumber(GOAL_HEADING_OFFSET_KEY, Math.toDegrees(GOAL_HEADING_OFFSET)));
    }

    @Override
    protected void initialize() {
        super.initialize();
        stale = VisionResults.isStale();
        headingAverage.reset(getAimHeading());
    }

    @Override
    protected boolean isFinished() {
        return super.isFinished() || stale;
    }
}
