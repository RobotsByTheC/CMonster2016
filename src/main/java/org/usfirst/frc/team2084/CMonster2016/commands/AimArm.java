/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.subsystems.ShooterSubsystem;
import org.usfirst.frc.team2084.CMonster2016.vision.VisionResults;

/**
 * Aims the arm to the angle given by the vision system. If the vision data is
 * stale, it immediately ends.
 */
public class AimArm extends SetArmAngle {

    /**
     * It should take a maximum of 5 seconds for the arm to aim. If it takes
     * longer than this, it probably means the error is slightly larger than the
     * tolerance.
     */
    public static final double TIMEOUT = 5;
    private boolean stale = false;

    public AimArm() {
        this(0);
    }

    public AimArm(double angleOffset) {
        super(() -> ShooterSubsystem.getCalibrationAngle(VisionResults.getGoalDistance()) + angleOffset);
        setTimeout(TIMEOUT);
    }

    /**
     * 
     */
    @Override
    protected void initialize() {
        super.initialize();

        stale = VisionResults.isStale();
    }

    /**
     * @return true when the arm gets to its setpoint or the operation times out
     */
    @Override
    protected boolean isFinished() {
        return super.isFinished() || isTimedOut() || stale;
    }
}
