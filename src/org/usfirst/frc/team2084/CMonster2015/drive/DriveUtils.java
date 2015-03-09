/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * This class contains some useful methods for driving.
 *
 * @author Ben Wolsieffer
 */
public final class DriveUtils {

    public static final double TWO_PI = Math.PI * 2;

    /**
     * Limits a value to the -1.0 to +1.0 range.
     *
     * @param value the number to limit
     * @return the limited number
     */
    public static double limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }

    /**
     * Checks if a value is between -1.0 and 1.0.
     *
     * @param value true if value {@literal >}=-1.0 and {@literal <}=1.0
     * @return
     */
    public static boolean isValid(double value) {
        return value >= -1.0 && value <= 1.0;
    }

    /**
     * Rotate a vector in Cartesian space.
     *
     * @param x the x component of the vector
     * @param y the y component of the vector
     * @param angle the angle (in radians) to rotate the vector
     * @return a 2 element array containing the rotated vector
     *
     */
    public static double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle);
        double sinA = Math.sin(angle);
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

    /**
     * Normalizes a heading value between -pi and pi.
     * 
     * @param heading the raw heading
     * @return the normalized heading
     */
    public static double normalizeHeading(double heading) {
        return heading - TWO_PI * Math.floor((heading + Math.PI) / TWO_PI);
    }

    /**
     * Creates a {@link PIDController} from a {@link PIDConstants} object and
     * the specified source and output.
     * 
     * @param constants the PID constants
     * @param source the PID source
     * @param output the PID output
     * @return a shiny new PID controller
     */
    public static PIDController createPIDControllerFromConstants(PIDConstants constants, PIDSource source, PIDOutput output) {
        return new PIDController(constants.p, constants.i, constants.d, constants.f, source, output);
    }
}
