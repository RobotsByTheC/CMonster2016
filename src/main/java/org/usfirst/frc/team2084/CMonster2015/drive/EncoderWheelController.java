/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * @author Ben Wolsieffer
 */
public abstract class EncoderWheelController<S extends SpeedController> extends WheelController<S> {

    /**
     * Flag the stores whether the encoder is enabled.
     */
    private boolean encoderEnabled = true;

    /**
     * Creates a new {@link EncoderWheelController} that uses the specified
     * motors.
     * 
     * @param pdpPorts the PDP ports the motors are connected to
     * @param motors the motors that make up the wheel controller
     */
    @SafeVarargs
    public EncoderWheelController(int[] pdpPorts, S... motors) {
        super(pdpPorts, motors);
    }

    /**
     * Get the distance traveled by this wheel (usually in meters).
     * 
     * @return the distance traveled
     */
    public abstract double getDistance();

    /**
     * Get the speed at which this wheel is traveling (usually in
     * meters/second).
     * 
     * @return the linear speed of the wheel
     */
    public abstract double getSpeed();

    /**
     * Resets the wheel controller. This will do different things in different
     * implementations.
     */
    public abstract void reset();

    /**
     * Gets whether the encoder is enabled.
     * 
     * @return true if the encoder is enabled
     */
    public boolean isEncoderEnabled() {
        return encoderEnabled;
    }

    /**
     * Sets whether the encoder is enabled. This should be used to fall back to
     * limited functionality in case the encoder fails.
     * 
     * @param enabled whether the encoder should be enabled
     */
    public void setEncoderEnabled(boolean enabled) {
        encoderEnabled = enabled;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void updateTable(ITable table) {
        super.updateTable(table);
        table.putNumber("Speed (m per s)", getSpeed());
        table.putNumber("Distance (m)", getDistance());
    }
}
