/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
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
public class TempSensor extends SensorBase {

	private final AnalogOutput analog;
	private double voltsPerDegree = 0.009;
	private double calibrationVolts = 2.5;
	private double calibrationTemperature = 25;

	public TempSensor(int channel) {
		analog = new AnalogOutput(channel);
	}

	public TempSensor(AnalogOutput channel) {
		analog = channel;
	}

	public void setVoltsPerDegree(double vpd) {
		voltsPerDegree = vpd;
	}

	public void setCalibration(double temperature, double voltage) {
		calibrationVolts = voltage;
		calibrationTemperature = temperature;
	}

	public double getVoltsPerDegree() {
		return voltsPerDegree;
	}

	public double getCalibrationVoltage() {
		return calibrationVolts;
	}

	public double getCalibrationTemperature() {
		return calibrationTemperature;
	}

	public double getTemperature() {
		return calibrationTemperature + ((analog.getVoltage() - calibrationVolts) * voltsPerDegree);
	}
}
