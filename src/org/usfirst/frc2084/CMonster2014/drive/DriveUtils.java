/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive;

import edu.wpi.first.wpilibj.parsing.IUtility;

/**
 * This class contains some useful methods for driving.
 *
 * @author Ben Wolsieffer
 */
public final class DriveUtils implements IUtility {

    /**
     * Limits a value to the -1.0 to +1.0 range.
     *
     * @param num the number to limit
     * @return the limited number
     */
    public static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    /**
     * Rotate a vector in Cartesian space.
     *
     * @param x the x component of the vector
     * @param y the x component of the vector
     * @param angle the angle to rotate the vector
     * @return a 2 element array containing the rotated vector
     *
     */
    public static double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle * (3.14159 / 180.0));
        double sinA = Math.sin(angle * (3.14159 / 180.0));
        double out[] = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        return out;
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than
     * 1.0.
     *
     * @param wheelSpeeds the array of wheel speeds to normalize
     */
    public static void normalize(double wheelSpeeds[]) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        int i;
        // Loops through each number to find the beggest absolute value.
        for (i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        // If the maximum is greater than 1.0, reduce all the values down 
        // proportionally so the maximum becomes 1.0.
        if (maxMagnitude > 1.0) {
            for (i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }
}
