/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Controls a single wheel with an arbitrary number of motors. In many cases
 * this will actually control a single wheel, but in designs where multiple
 * wheels are connected to the same gearbox (such as most skid steer systems),
 * it will actually control three to four wheels.
 *
 * @author Ben Wolsieffer
 */
public class WheelController<S extends SpeedController> implements Sendable {

    /**
     * A PDP object used to get current usage
     */
    protected static final PowerDistributionPanel pdp = new PowerDistributionPanel();

    /**
     * Array of PDP ports that the motors are connected to. They are in the same
     * order as {@link #motors}.
     */
    protected final int[] pdpPorts;

    /**
     * Array of motors that make up this wheel controller.
     */
    protected final S[] motors;
    /**
     * Multiplied by the speed value, so to invert the motor, this is set to -1.
     */
    private double inverted = 1;

    private double value = 0;

    /**
     * Creates a new {@link WheelController} with an arbitrary number of motors.
     *
     * @param pdpPorts the PDP ports the motors are connected to
     * @param motors the array of speed controllers that power the wheel
     */
    @SafeVarargs
    public WheelController(int[] pdpPorts, S... motors) {
        this.motors = motors;
        this.pdpPorts = pdpPorts;
    }

    /**
     * Sets the speed of the wheel. For a basic wheel this just sets the power
     * to the motor but this class can be extended for more complex
     * functionality.
     *
     * @param speed the wheel speed between 1.0 and -1.0
     */
    public void set(double speed) {
        value = speed;
        if (table != null) {
            updateTable(table);
        }
        speed = DriveUtils.limit(speed) * inverted;
        for (int i = 0; i < motors.length; i++) {
            motors[i].set(speed);
        }
    }

    public double get() {
        return value;
    }

    /**
     * Sets the inversion of all the motors of this wheel.
     *
     * @param inverted whether the wheel is inverted
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted ? -1 : 1;
    }

    /**
     * Gets whether the wheel motors are inverted.
     * 
     * @return whether the wheel is inverted
     */
    public boolean isInverted() {
        return inverted == -1;
    }

    /**
     * Get the total current used by all the motors of this wheel controller.
     * 
     * @return the total current draw
     */
    public double getCurrent() {
        double current = 0;
        for (int pdpPort : pdpPorts) {
            current += pdp.getCurrent(pdpPort);
        }
        return current;
    }

    /*
     * SmartDashboard type.
     */
    @Override
    public String getSmartDashboardType() {
        return "Wheel Controller";
    }

    private ITable table;

    /**
     * {@inheritDoc}
     */
    @Override
    public void initTable(ITable subtable) {
        table = subtable;
        if (table != null) {
            updateTable(table);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public ITable getTable() {
        return table;
    }

    public void updateTable(ITable table) {
        table.putNumber("Current (amps)", getCurrent());
        table.putNumber("Value", get());
    }
}
