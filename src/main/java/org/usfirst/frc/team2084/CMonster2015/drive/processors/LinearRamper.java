/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive.processors;

import edu.wpi.first.wpilibj.Timer;

/**
 * Ramps a value by a certain amount per second. If the input increases at a
 * rate that is faster than the ramp rate, it will be reduced to ramp at the
 * correct rate. The ramper can be selected to ramp up, down or both.
 * 
 * @author Ben Wolsieffer
 */
public class LinearRamper implements Processor {

    /**
     * The type of ramping.
     */
    public static enum Type {
        /**
         * Ramp up values only.
         */
        UP,
        /**
         * Ramp down values only.
         */
        DOWN,
        /**
         * Ramp up and down.
         */
        UP_DOWN
    }

    /**
     * The type of ramping.
     * 
     * @see Type
     */
    private final Type type;
    /**
     * The rate of ramping.
     */
    private final double rampRate;
    /**
     * The output value when {@link #process(double)} was last called.
     */
    private double lastValue;
    /**
     * The time in seconds when {@link #process(double)} was last called.
     */
    private double lastTime;

    /**
     * Creates a new {@link LinearRamper} with the specified ramp rate and
     * {@link Type}.
     * 
     * @param rampRate the maximum ramp in units/second
     * @param type whether the ramper should ramp up, down or both
     */
    public LinearRamper(double rampRate, Type type) {
        this.rampRate = Math.abs(rampRate);
        this.type = type;
        reset();
    }

    /**
     * Ramps the input value.
     * 
     * @param value input value
     * @return the ramped value
     */
    @Override
    public double process(double value) {
        double currTime = Timer.getFPGATimestamp();
        double elapsedTime = currTime - lastTime;
        lastTime = currTime;

        double delta = value - lastValue;
        // Maximum change in the value between the previous and current
        double maxDelta = rampRate * elapsedTime;
        double output = value;
        // If the value increased too fast, limit it
        if (Math.abs(delta) > maxDelta) {
            delta = maxDelta * (delta < 0 ? -1 : 1);
            value = lastValue + delta;
            if (delta < 0) {
                if (output < 0) {
                    if (type == Type.UP) {
                        output = value;
                    }
                } else {
                    if (type == Type.DOWN) {
                        output = value;
                    }
                }
            } else {
                if (output < 0) {
                    if (type == Type.DOWN) {
                        output = value;
                    }
                } else {
                    if (type == Type.UP) {
                        output = value;
                    }
                }
            }
        }
        lastValue = output;
        return output;
    }

    /**
     * Reset the last value and time.
     */
    @Override
    public void reset() {
        lastValue = 0;
        lastTime = Timer.getFPGATimestamp();
    }
}
