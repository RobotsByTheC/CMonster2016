/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * @author Ben Wolsieffer
 */
public class DIOEncoderWheelController extends EncoderWheelController<SpeedController> {

	private final PIDController distancePIDController;
	private final PIDController speedPIDController;

	private double pidOutput = 0;

	private final Encoder encoder;

	/**
	 * 
	 */
	public DIOEncoderWheelController(Encoder encoder, PIDConstants speedPIDConstants, PIDConstants distancePIDConstants, SpeedController... motors) {
		super(motors);
		speedPIDController = new PIDController(
				speedPIDConstants.p,
				speedPIDConstants.i,
				speedPIDConstants.d,
				speedPIDConstants.f,
				encoder::getRate, (o) -> {
					pidOutput = o;
				}
				);

		distancePIDController = new PIDController(
				distancePIDConstants.p,
				distancePIDConstants.i,
				distancePIDConstants.d,
				distancePIDConstants.f,
				encoder::getDistance, speedPIDController::setSetpoint
				);
		this.encoder = encoder;
	}

	@Override
	public void rotateToDistance(double distance) {
		distancePIDController.enable();
		distancePIDController.setSetpoint(distance);
		super.set(pidOutput);
	}

	@Override
	public void set(double speed) {
		if (isEncoderEnabled()) {
			distancePIDController.disable();
			speedPIDController.setSetpoint(speed);
			super.set(pidOutput);
		} else {
			super.set(speed);
		}
	}

	@Override
	public double getDistance() {
		return encoder.getDistance();
	}

	@Override
	public double getSpeed() {
		return encoder.getRate();
	}

	@Override
	public void reset() {
		speedPIDController.reset();
		distancePIDController.reset();
		if (isEncoderEnabled()) {
			speedPIDController.enable();
		}
	}

	@Override
	public void setEncoderEnabled(boolean enabled) {
		if (enabled != isEncoderEnabled()) {
			if (enabled) {
				reset();
			} else {
				speedPIDController.disable();
				distancePIDController.disable();
			}
		}
		super.setEncoderEnabled(enabled);
	}
}
