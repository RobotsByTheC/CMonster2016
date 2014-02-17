/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 *//*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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
