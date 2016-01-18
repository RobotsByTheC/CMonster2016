/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.drive;

/**
 * Represents the constants of a PID controller.
 * 
 * @author Ben Wolsieffer
 */
public class PIDConstants {

    public final double p;
    public final double i;
    public final double d;
    public final double f;

    /**
     * Create new {@link PIDConstants} object with the specified constants.
     * 
     * @param p the proportional constant
     * @param i the integral constant
     * @param d the differential constant
     * @param f the feed-forward constant
     */
    public PIDConstants(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
    }

    /**
     * Create new {@link PIDConstants} object with the specified constants and 0
     * for the feed-forward constant.
     * 
     * @param p the proportional constant
     * @param i the integral constant
     * @param d the differential constant
     */
    public PIDConstants(double p, double i, double d) {
        this(p, i, d, 0);
    }
}
