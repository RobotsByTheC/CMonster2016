/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive.processors;

/**
 * Scales a value by a constant.
 *
 * @author Ben Wolsieffer
 */
public class Scaler implements Processor {

    /**
     * The scaling constant.
     */
    private final double scale;

    /**
     * Creates a {@link Scaler} that multiplies a value by a constant.
     * 
     * @param scale the multiplier
     */
    public Scaler(double scale) {
        this.scale = scale;
    }

    /**
     * Scales the input value.
     * 
     * @param value the input value
     * @return the scaled value
     */
    @Override
    public double process(double value) {
        return value * scale;
    }
}
