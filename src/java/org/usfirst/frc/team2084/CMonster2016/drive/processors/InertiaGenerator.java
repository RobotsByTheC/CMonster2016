/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.drive.processors;

import edu.wpi.first.wpilibj.Timer;

/**
 * Generates inertia for an input value. This makes the robot respond faster to
 * quick changes in input. Functionally, it acts like a PD controller with a
 * {@code P=1.0}, {@code D=inertiaGain} and {@code error=value}. This idea was
 * taken from Team 254 (The Cheesy Poofs), but I implemented it somewhat
 * differently.
 *
 * @author Ben Wolsieffer
 */
public class InertiaGenerator implements Processor {

    /**
     * The output value the last time {@link #process(double)} was called.
     */
    private double lastValue;
    /**
     * The time in seconds the last time {@link #process(double)} was called.
     */
    private double lastTime;

    /**
     * The inertia gain constant.
     */
    private final double inertiaGain;

    /**
     * Creates a new {@link InertiaGenerator} with the specified inertia gain.
     * 
     * @param inertiaGain the inertia gain constant
     */
    public InertiaGenerator(double inertiaGain) {
        this.inertiaGain = inertiaGain;
        reset();
    }

    /**
     * Generates inertia based on an input.
     * 
     * @param value the input value
     * @return the inertia
     */
    @Override
    public double process(double value) {
        double currTime = Timer.getFPGATimestamp();
        double elapsedTime = currTime - lastTime;
        lastTime = currTime;

        // Get the difference between current and previous values
        double output = value - lastValue;
        // Normalize based on elapsed time
        output /= elapsedTime;
        // Multiply the normalized delta by the gain
        output *= inertiaGain;
        // Update the last value
        lastValue = value;
        return value + output;
    }

    /**
     * Resets the last output and time.
     */
    @Override
    public void reset() {
        lastValue = 0;
        lastTime = Timer.getFPGATimestamp();
    }
}
