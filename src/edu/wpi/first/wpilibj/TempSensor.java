/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.parsing.ISensor;

/**
 *
 * @author Ben Wolsieffer
 */
public class TempSensor implements ISensor {

    private final AnalogChannel analog;
    private double voltsPerDegree = 0.009;
    private double nominalVolts = 2.5;

    public TempSensor(int slot, int channel) {
        analog = new AnalogChannel(slot, channel);
    }

    public TempSensor(int channel) {
        analog = new AnalogChannel(channel);
    }

    public TempSensor(AnalogChannel channel) {
        analog = channel;
    }

    public void setVoltsPerDegree(double vpd) {
        voltsPerDegree = vpd;
    }

    public void setNominalVoltage(double nv) {
        nominalVolts = nv;
    }

    public double getVoltsPerDegree() {
        return voltsPerDegree;
    }

    public double getNominalVoltage() {
        return nominalVolts;
    }

    public double getTemp() {
        return (analog.getAverageVoltage() - nominalVolts) * voltsPerDegree;
    }
}
