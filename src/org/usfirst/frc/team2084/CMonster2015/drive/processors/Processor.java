/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive.processors;

/**
 * An interface that takes an input, processes it and outputs a different value.
 *
 * @author Ben Wolsieffer
 */
@FunctionalInterface
public interface Processor {

    /**
     * Processes an input value to produce an output.
     * 
     * @param value the input value
     * @return the output value
     */
    double process(double value);

    /**
     * Resets the {@link Processor}.
     */
    default void reset() {
    }
}
