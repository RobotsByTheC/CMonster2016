/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import edu.wpi.first.wpilibj.CANTalon;

/**
 * Wheel controller that uses a CANTalon with an attached quadrature encoder to
 * regulate speed.
 * 
 * @author Ben Wolsieffer
 */
public class CANTalonEncoderWheelController extends EncoderWheelController<CANTalon> {

    public final double encoderDistancePerPulse;
    public final double maxSpeed;

    private final CANTalon masterMotor;

    /**
     * Creates a {@link CANTalonEncoderWheelController} that uses the specified
     * PID constants, max speed and distance per encoder pulse. It also takes a
     * list of motors that form this wheel controller. The first Talon SRX
     * serves as the master and it does the PID calculation, while the rest
     * follow its output. This means that the encoder should be attached to the
     * first Talon in the list.
     * 
     * @param speedPIDConstants the constants for the speed PID controller
     * @param maxSpeed the maximum speed the wheel can turn
     * @param encoderDistancePerPulse the distance the wheel moves per encoder
     *            pulse
     * @param pdpPorts the PDP ports the motors are connected to
     * @param motors the list of motors that make up this wheel controller
     */
    public CANTalonEncoderWheelController(PIDConstants speedPIDConstants,
            double maxSpeed, double encoderDistancePerPulse, int[] pdpPorts, CANTalon... motors) {
        super(pdpPorts, motors);

        this.maxSpeed = maxSpeed;
        this.encoderDistancePerPulse = encoderDistancePerPulse;

        // Master controller does all the controlling while the others are
        // slaves.
        masterMotor = motors[0];
        // Set all other motors as slaves
        for (int i = 1; i < motors.length; i++) {
            motors[i].changeControlMode(CANTalon.ControlMode.Follower);
            // Setup the motors to follow the master
            motors[i].set(motors[0].getDeviceID());
        }

        masterMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);

        // Speed profile (using 0 ramp value to disable ramping)
        masterMotor.setPID(speedPIDConstants.p, speedPIDConstants.i, speedPIDConstants.d,
                speedPIDConstants.f, 0, 0, 0);
    }

    /**
     * Converts an encoder count value to distance units.
     * 
     * @param count the encoder count
     * @return the corresponding distance
     */
    private double convertCountToDistance(double count) {
        return count * encoderDistancePerPulse;
    }

    /**
     * Converts a distance to an encoder count.
     * 
     * @param distance the distance
     * @return the corresponding encoder count
     */
    private double convertDistanceToCount(double distance) {
        return distance / encoderDistancePerPulse;
    }

    /**
     * Converts a throttle value (-1.0 to 1.0) to an encoder count per 10ms
     * speed, which the Talon uses internally.
     * 
     * @param throttle the throttle value
     * @return the corresponding encoder count per 10 milliseconds
     */
    private double convertThrottleToCountPer10ms(double throttle) {
        return convertDistanceToCount(maxSpeed * throttle) / 100;
    }

    /**
     * Sets the speed of this wheel controller. This is maintained using a PID
     * controller inside the Talon and an encoder.
     * 
     * @param speed the speed of the wheel (-1.0 to 1.0)
     */
    @Override
    public void set(double speed) {
        if (isEncoderEnabled()) {
            masterMotor.changeControlMode(CANTalon.ControlMode.Speed);
            masterMotor.set(convertThrottleToCountPer10ms(speed));
        } else {
            masterMotor.changeControlMode(CANTalon.ControlMode.PercentVbus);
            masterMotor.set(speed);
        }
    }

    /**
     * Gets the distance the wheel has traveled, usually in meters.
     * 
     * @return the distance
     */
    @Override
    public double getDistance() {
        return convertCountToDistance(masterMotor.getEncPosition());
    }

    /**
     * Gets the speed the wheel is traveling in distance units (usually meters)
     * per second.
     * 
     * @return the wheel distance
     */
    @Override
    public double getSpeed() {
        return convertCountToDistance(masterMotor.getEncVelocity());
    }

    /**
     * Inverts the encoder readings.
     * 
     * @param inverted whether the encoder should be inverted
     */
    public void setEncoderInverted(boolean inverted) {
        masterMotor.reverseSensor(inverted);
    }

    /**
     * Inverts the output of the wheel.
     * 
     * @param inverted whether the wheel should be inverted
     */
    @Override
    public void setInverted(boolean inverted) {
        masterMotor.reverseOutput(inverted);
    }

    /**
     * Resets the speed PID controller.
     */
    @Override
    public void reset() {
        masterMotor.ClearIaccum();
    }
}
