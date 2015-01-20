/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj;

/**
 * A gyro that reports its reading in radians rather than degrees. It also uses
 * a software, rather than hardware reset, in an attempt to prevent jumps in the
 * signal.
 *
 * @author Ben Wolsieffer
 */
public class SynchronizedRadianGyro extends Gyro {

	/**
	 * Stores the gyro zero value for the software reset.
	 */
	public double resetOffset = 0.0;

	public SynchronizedRadianGyro(int channel) {
		super(channel);
	}

	public SynchronizedRadianGyro(AnalogInput channel) {
		super(channel);
	}

	/**
	 * Return the rate of rotation of the gyro. The rate is based on the most
	 * recent reading of the gyro analog value.
	 *
	 * @return the current rate in radians per second
	 */
	@Override
	public double getRate() {
		synchronized (this) {
			return Math.toRadians(super.getRate());
		}
	}

	/**
	 * Return the actual angle in radians that the robot is currently facing.
	 *
	 * The angle is based on the current accumulator value corrected by the
	 * oversampling rate, the gyro type and the A/D calibration values. The
	 * angle is continuous, that is can go beyond 360 degrees.
	 *
	 * @return the current heading of the robot in radians
	 */
	@Override
	public double getAngle() {
		return getRawAngle() - resetOffset;
	}

	private double getRawAngle() {
		synchronized (this) {
			return Math.toRadians(super.getAngle());
		}
	}

	@Override
	public void reset() {
		// Used in an attempt to prevent jerking as the accumulator is reset,
		// do a software reset instead. I'm not sure if we really need it.
		resetOffset = getRawAngle();
	}
}
