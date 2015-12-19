/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj;

/**
 * A temperature sensor. This was mostly made for fun to display the value of
 * the gyro temperature sensor. Theoretically it could be used to calibrate the
 * gyro.
 *
 * @author Ben Wolsieffer
 */
public class TemperatureSensor extends SensorBase {

    public static final double DEFAULT_VOLTS_PER_DEGREE = 0.009;
    public static final double DEFAULT_CALIBRATION_VOLTAGE = 2.5;
    public static final double DEFAULT_CALIBRATION_TEMPERATURE = 25;

    private final AnalogInput analog;
    private double voltsPerDegree = DEFAULT_VOLTS_PER_DEGREE;
    private double calibrationVoltage = DEFAULT_CALIBRATION_VOLTAGE;
    private double calibrationTemperature = DEFAULT_CALIBRATION_TEMPERATURE;

    /**
     * Creates a new temperature sensor on the specified analog channel.
     * 
     * @param channel the channel number
     */
    public TemperatureSensor(int channel) {
        analog = new AnalogInput(channel);
    }

    /**
     * Creates a new temperature sensor on the precreated analog channel. This
     * can be used if the channel needs to be shared.
     * 
     * @param input the analog input channel
     */
    public TemperatureSensor(AnalogInput input) {
        analog = input;
    }

    /**
     * Sets the number of volts per degree celcius that the temperature sensor
     * outputs.
     * 
     * @param vpd the volts per degree celcius
     */
    public void setVoltsPerDegree(double vpd) {
        voltsPerDegree = vpd;
    }

    /**
     * Sets the calibration values of the sensor. This consists of a temperature
     * and its corresponding voltage.
     * 
     * @param temperature the calibration temperature
     * @param voltage the calibration voltage
     */
    public void setCalibration(double temperature, double voltage) {
        calibrationVoltage = voltage;
        calibrationTemperature = temperature;
    }

    /**
     * Gets the volts per degree celcius that the sensor outputs.
     * 
     * @return the volts per degree
     */
    public double getVoltsPerDegree() {
        return voltsPerDegree;
    }

    /**
     * Gets the calibration voltage of the sensor.
     * 
     * @see #setCalibration(double, double)
     * @return the calibration voltage
     */
    public double getCalibrationVoltage() {
        return calibrationVoltage;
    }

    /**
     * Gets the calibration temperature of the sensor.
     * 
     * @see #setCalibration(double, double)
     * @return the calibration temperature
     */
    public double getCalibrationTemperature() {
        return calibrationTemperature;
    }

    /**
     * Gets the current temperature from the sensor.
     * 
     * @return the current temperature in degrees celcius
     */
    public double getTemperature() {
        return calibrationTemperature
                + ((analog.getVoltage() - calibrationVoltage) * voltsPerDegree);
    }
}
