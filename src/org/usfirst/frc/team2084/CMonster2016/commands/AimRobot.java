/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.vision.VisionResults;

/**
 * Aims the robot to the heading given by the vision system. If the vision data
 * is stale, it immediately ends.
 */
public class AimRobot extends RotateToHeading {

    public static final double TIMEOUT = 5;
    private boolean stale = false;

    public AimRobot() {
        super(() -> VisionResults.getGoalHeading() + Math.toRadians(3));
        setTimeout(TIMEOUT);
    }

    @Override
    protected void initialize() {
        super.initialize();
        stale = VisionResults.isStale();
    }

    @Override
    protected boolean isFinished() {
        return super.isFinished() || isTimedOut() || stale;
    }
}
