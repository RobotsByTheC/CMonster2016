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
 * A {@link WheelController} that uses an encoder attached to digital IO pins
 * and any speed controller to maintain speed. This runs the PID controller on
 * the roboRIO.
 * 
 * @author Ben Wolsieffer
 */
public class DIOEncoderWheelController<S extends SpeedController> extends EncoderWheelController<SpeedController> {

    /**
     * The PIDController the maintains the speed of the wheel.
     */
    private final PIDController speedPIDController;

    /**
     * The output of the {@link #speedPIDController}.
     */
    private double pidOutput = 0;

    /**
     * The encoder that is used as a feedback device.
     */
    private final Encoder encoder;

    /**
     * The maximum speed the wheel is capable of moving.
     */
    private final double maxSpeed;

    /**
     * Creates a new {@link DIOEncoderWheelController} using the specified
     * encoder, PID constants and speed controllers.
     * 
     * @param encoder the encoder to use for closed loop control
     * @param speedPIDConstants the PID constants to use for the speed control
     *            loop
     * @param pdpPorts the PDP ports the motors are connected to
     * @param motors the list of motors to control
     */
    @SafeVarargs
    public DIOEncoderWheelController(Encoder encoder, PIDConstants speedPIDConstants, double maxSpeed, int[] pdpPorts, S... motors) {
        super(pdpPorts, motors);
        speedPIDController = DriveUtils.createPIDControllerFromConstants(speedPIDConstants,
                encoder::getRate, (o) -> pidOutput = o);
        speedPIDController.enable();
        this.maxSpeed = maxSpeed;

        this.encoder = encoder;
    }

    /**
     * Sets the speed of this wheel controller. This is maintained using a PID
     * controller and an encoder.
     * 
     * @param speed the speed of the wheel (-1.0 to 1.0)
     */
    @Override
    public void set(double speed) {
        if (isEncoderEnabled()) {
            speedPIDController.setSetpoint(speed * maxSpeed);
            super.set(pidOutput);
        } else {
            super.set(speed);
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getDistance() {
        return encoder.getDistance();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public double getSpeed() {
        return encoder.getRate();
    }

    /**
     * Resets the speed PID controller.
     */
    @Override
    public void reset() {
        speedPIDController.reset();
        if (isEncoderEnabled()) {
            speedPIDController.enable();
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setEncoderEnabled(boolean enabled) {
        super.setEncoderEnabled(enabled);
        if (enabled != isEncoderEnabled()) {
            if (enabled) {
                reset();
            } else {
                speedPIDController.disable();
            }
        }
    }
}
