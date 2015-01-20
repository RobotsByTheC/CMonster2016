/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;

/**
 * @author Ben Wolsieffer
 */
public class CANTalonEncoderWheelController extends EncoderWheelController<CANTalon> {

	public static final int POSITION_PROFILE = 0;

	public static final int SPEED_PROFILE = 1;

	public final double encoderDistancePerPulse;

	public final double maxSpeed;

	private final CANTalon masterMotor;

	private CANTalon.ControlMode controlMode = ControlMode.Position;

	public CANTalonEncoderWheelController(PIDConstants distancePIDConstants, PIDConstants speedPIDConstants,
			double maxSpeed, double encoderDistancePerPulse, CANTalon... motors) {
		super(motors);

		this.maxSpeed = maxSpeed;
		this.encoderDistancePerPulse = encoderDistancePerPulse;

		// Master controller does all the controlling while the others are
		// slaves.
		masterMotor = motors[0];
		// Set all other motors as slaves
		for (int i = 1; i < motors.length; i++) {
			motors[i].changeControlMode(CANTalon.ControlMode.Follower);
		}

		masterMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

		// Setup PID profiles
		// Position profile (using >12 ramp value to disable ramping)
		masterMotor.setPID(distancePIDConstants.p, distancePIDConstants.i, distancePIDConstants.d, distancePIDConstants.f, 0, 20, POSITION_PROFILE);
		// Speed profile (using >12 ramp value to disable ramping)
		masterMotor.setPID(speedPIDConstants.p, speedPIDConstants.i, speedPIDConstants.d, speedPIDConstants.f, 0, 20, SPEED_PROFILE);

		// Use speed control by default
		setControlMode(CANTalon.ControlMode.Speed);
	}

	private void setControlMode(CANTalon.ControlMode mode) {
		if (mode != controlMode) {
			switch (mode) {
			default:
			case PercentVbus:
				break;
			case Speed:
				masterMotor.setProfile(SPEED_PROFILE);
				break;
			case Position:
				masterMotor.setProfile(POSITION_PROFILE);
				break;
			}
			masterMotor.changeControlMode(mode);
			controlMode = mode;
		}
	}

	private double convertCountToDistance(double count) {
		return count * encoderDistancePerPulse;
	}

	private double convertDistanceToCount(double distance) {
		return distance / encoderDistancePerPulse;
	}

	private double convertThrottleToDistancePer10ms(double throttle) {
		return (maxSpeed * throttle) / 100;
	}

	@Override
	public void set(double speed) {
		if (isEncoderEnabled()) {
			setControlMode(CANTalon.ControlMode.Speed);
			masterMotor.set(convertThrottleToDistancePer10ms(speed));
		} else {
			setControlMode(CANTalon.ControlMode.PercentVbus);
			masterMotor.set(speed);
		}
	}

	@Override
	public void rotateToDistance(double position) {
		setControlMode(CANTalon.ControlMode.Position);
		masterMotor.set(convertDistanceToCount(position));
	}

	@Override
	public double getDistance() {
		return convertCountToDistance(masterMotor.getEncPosition());
	}

	@Override
	public double getSpeed() {
		return masterMotor.getEncVelocity();
	}

	public void setEncoderInverted(boolean inverted) {
		masterMotor.reverseSensor(inverted);
	}

	@Override
	public void setInverted(boolean inverted) {
		masterMotor.reverseOutput(inverted);
	}

	@Override
	public void reset() {
		masterMotor.ClearIaccum();
	}

}
