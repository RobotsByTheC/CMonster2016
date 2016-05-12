/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.drive;

import org.usfirst.frc.team2084.CMonster2016.drive.processors.LinearRamper;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * A {@link WheelController} that uses an encoder attached to digital IO pins
 * and any speed controller to maintain speed. This runs the PID controller on
 * the roboRIO.
 * 
 * @author Ben Wolsieffer
 */
@Parameter(key = DIOEncoderWheelController.P_KEY, type = Type.NUMBER, numberValue = 0)
@Parameter(key = DIOEncoderWheelController.I_KEY, type = Type.NUMBER, numberValue = 0)
@Parameter(key = DIOEncoderWheelController.D_KEY, type = Type.NUMBER, numberValue = 0)
@Parameter(key = DIOEncoderWheelController.PID_RAMP_RATE_KEY, type = Type.NUMBER,
        numberValue = DIOEncoderWheelController.DEFAULT_PID_RAMP_RATE)
@Parameter(key = DIOEncoderWheelController.DEBUG_KEY, type = Type.BOOLEAN, booleanValue = false)
public class DIOEncoderWheelController<S extends SpeedController> extends EncoderWheelController<S> {

    public static final String P_KEY = "p";
    public static final String I_KEY = "i";
    public static final String D_KEY = "d";
    public static final String PID_RAMP_RATE_KEY = "ramp_rate";
    public static final String DEBUG_KEY = "debug";

    public static final double DEFAULT_PID_RAMP_RATE = 3;

    private final LinearRamper ramper = new LinearRamper(DEFAULT_PID_RAMP_RATE, LinearRamper.Type.UP);

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

    @SuppressWarnings("rawtypes")
    private final ParameterBundle<DIOEncoderWheelController> parameters;
    private final String debugKey;

    /**
     * Creates a new {@link DIOEncoderWheelController} using the specified
     * encoder, PID constants and speed controllers.
     * 
     * @param encoder the encoder to use for closed loop control
     * @param speedPIDConstants the PID constants to use for the speed control
     *        loop
     * @param maxSpeed the maximum allowable output speed
     * @param pdpPorts the PDP ports the motors are connected to
     * @param motors the list of motors to control
     */
    @SafeVarargs
    public DIOEncoderWheelController(Encoder encoder,
            @SuppressWarnings("rawtypes") ParameterBundle<DIOEncoderWheelController> parameters, String debugKey,
            double maxSpeed, int[] pdpPorts, S... motors) {
        super(pdpPorts, motors);
        this.parameters = parameters;
        this.debugKey = debugKey;
        speedPIDController = new PIDController(0, 0, 0, new PIDSource() {

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
            }

            @Override
            public double pidGet() {
                return encoder.getRate();
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kRate;
            }
        }, (o) -> pidOutput = o);

        parameters.addListener((key, type, val) -> {
            switch (key) {
            case P_KEY:
            case I_KEY:
            case D_KEY:
                double p = parameters.getNumber(P_KEY);
                double i = parameters.getNumber(I_KEY);
                double d = parameters.getNumber(D_KEY);
                speedPIDController.setPID(p, i, d);
            break;
            case PID_RAMP_RATE_KEY:
                ramper.setRampRate((Double) val);
            break;
            }
        });

        speedPIDController.enable();
        this.maxSpeed = maxSpeed;

        this.encoder = encoder;
    }

    double[] debugPID = new double[3];

    /**
     * Sets the speed of this wheel controller. This is maintained using a PID
     * controller and an encoder.
     * 
     * @param speed the speed of the wheel (-1.0 to 1.0)
     */
    @Override
    public void set(double speed) {
        if (isEncoderEnabled()) {
            double setpoint = speed * maxSpeed;
            speedPIDController.setSetpoint(setpoint);
            super.set(ramper.process(pidOutput));

            if (parameters.getBoolean(DEBUG_KEY)) {
                debugPID[0] = Timer.getFPGATimestamp();
                debugPID[1] = setpoint;
                debugPID[2] = getSpeed();
                NetworkTable.getTable("").putNumberArray(debugKey, debugPID);
            }
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
     * 
     */
    @Override
    public void resetEncoder() {
        encoder.reset();
    }

    /**
     * Resets the speed PID controller.
     */
    @Override
    public void reset() {
        speedPIDController.reset();
        pidOutput = 0;
        if (isEncoderEnabled()) {
            speedPIDController.enable();
        }
        ramper.reset();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void setEncoderEnabled(boolean enabled) {
        boolean old = isEncoderEnabled();
        super.setEncoderEnabled(enabled);
        if (enabled != old) {
            if (enabled) {
                reset();
            } else {
                speedPIDController.disable();
            }
        }
    }
}
