/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016;

import java.util.Arrays;

/**
 * @author Ben Wolsieffer
 */
public class RollingAverage {

    private final double[] buffer;
    private final int numSamples;

    private double sum = 0;
    private int position;

    public RollingAverage(int numSamples) {
        this.numSamples = numSamples;
        buffer = new double[numSamples];
    }

    public void newValue(double value) {
        sum -= buffer[position];
        buffer[position] = value;
        sum += value;

        ++position;
        position %= numSamples;
    }

    public double getAverage() {
        return sum / (double)numSamples;
    }

    public void reset(double value) {
        Arrays.fill(buffer, value);
        sum = value * numSamples;
    }

    public void reset() {
        reset(0);
    }
}
