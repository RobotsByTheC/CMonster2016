/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015;

import edu.wpi.first.wpilibj.AccumulatorResult;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * An analog gyro that is attached to the analog input on the roboRIO. It has a
 * modifiable sensitivity to be able to support different gyros.
 * 
 * @author Ben Wolsieffer
 */
public class AnalogGyro extends SensorBase implements LiveWindowSendable, Gyro {

    /**
     * Default number of oversample bits.
     */
    public static final int DEFAULT_OVERSAMPLE_BITS = 10;
    /**
     * Default number of average bits.
     */
    public static final int DEFAULT_AVERAGE_BITS = 0;
    /**
     * Default sample rate in hertz.
     */
    public static final double DEFAULT_SAMPLES_PER_SECOND = 50.0;
    /**
     * Default calibration time in seconds.
     */
    public static final double DEFAULT_CALIBRATION_TIME = 5.0;
    /**
     * Default volts per radian per second calibration value.
     */
    public static final double DEFAULT_VOLTS_PER_RADIAN_PER_SECOND = 0.4010;
    /**
     * The analog input which the gyro is connected to.
     */
    protected AnalogInput analogInput;
    /**
     * The sample rate of the ADC.
     */
    private double sampleRate = DEFAULT_SAMPLES_PER_SECOND;
    /**
     * Volts per radian per second calibration value.
     */
    private double voltsPerRadianPerSecond = DEFAULT_VOLTS_PER_RADIAN_PER_SECOND;

    private double offset;
    private int center;

    /**
     * The offset used to set the gyro to a certain angle.
     */
    private double angleOffset = 0;

    /**
     * The result of the FPGA accumulator.
     */
    protected final AccumulatorResult result;

    /**
     * Calibrate the gyro by running for a number of samples and computing the
     * center value. Then use the center value as the Accumulator center value
     * for subsequent measurements. It's important to make sure that the robot
     * is not moving while the centering calculations are in progress, this is
     * automatically done when the robot is first turned on while it's sitting
     * at rest before the competition starts.
     */
    public void calibrate(double time) {
        reset();
        Timer.delay(time);

        analogInput.getAccumulatorOutput(result);

        center = (int) ((double) result.value / (double) result.count + .5);

        offset = ((double) result.value / (double) result.count) - center;

        analogInput.setAccumulatorCenter(center);
        reset();

        setDeadband(0.0);
    }

    /**
     * Gyro constructor using the channel number
     *
     * @param channel The analog channel the gyro is connected to. Gyros can
     *            only be used on on-board channels 0-1.
     */
    public AnalogGyro(int channel) {
        this(new AnalogInput(channel));
    }

    /**
     * Gyro constructor with a precreated analog channel object. Use this
     * constructor when the analog channel needs to be shared.
     *
     * @param channel The AnalogInput object that the gyro is connected to.
     *            Analog gyros can only be used on on-board channels 0-1.
     */
    public AnalogGyro(AnalogInput channel) {
        analogInput = channel;
        if (analogInput == null) {
            throw new NullPointerException("AnalogInput supplied to AnalogGyro constructor is null");
        }
        result = new AccumulatorResult();

        analogInput.setAverageBits(DEFAULT_AVERAGE_BITS);
        analogInput.setOversampleBits(DEFAULT_OVERSAMPLE_BITS);
        updateSampleRate();

        analogInput.initAccumulator();

        UsageReporting.report(tResourceType.kResourceType_Gyro, analogInput.getChannel(), 0, "Custom more flexible implementation");
        LiveWindow.addSensor("Analog Gyro", analogInput.getChannel(), this);

        calibrate(DEFAULT_CALIBRATION_TIME);
    }

    /**
     * Reset the gyro. Resets the gyro to a heading of zero. This can be used if
     * there is significant drift in the gyro and it needs to be reset after it
     * has been running.
     */
    @Override
    public void reset() {
        angleOffset = 0;
        analogInput.resetAccumulator();
    }

    /**
     * Return the actual angle in radians that the robot is currently facing.
     *
     * The angle is based on the current accumulator value corrected by the
     * oversampling rate, the gyro type and the A/D calibration values. The
     * angle is continuous, that is it will continue from past 2pi radians. This
     * allows algorithms that wouldn't want to see a discontinuity in the gyro
     * output as it sweeps past from 2pi to 0 on the second time around.
     *
     * @return the current heading of the robot in radians. This heading is
     *         based on integration of the returned rate from the gyro.
     */
    @Override
    public double getAngle() {
        analogInput.getAccumulatorOutput(result);

        long value = result.value - (long) (result.count * offset);

        double scaledValue = value
                * 1e-9
                * analogInput.getLSBWeight()
                * (1 << analogInput.getAverageBits())
                / (AnalogInput.getGlobalSampleRate() * voltsPerRadianPerSecond);

        return scaledValue + angleOffset;
    }

    @Override
    public void setAngle(double angle) {
        reset();
        angleOffset = angle;
    }

    /**
     * Return the rate of rotation of the gyro
     *
     * The rate is based on the most recent reading of the gyro analog value
     *
     * @return the current rate in radians per second
     */
    @Override
    public double getRate() {
        return (analogInput.getAverageValue() - (center + offset))
                * 1e-9
                * analogInput.getLSBWeight()
                / ((1 << analogInput.getOversampleBits()) * voltsPerRadianPerSecond);

    }

    /**
     * Set the gyro sensitivity. This takes the number of volts/radian/second
     * sensitivity of the gyro and uses it in subsequent calculations to allow
     * the code to work with multiple gyros. This value is typically found in
     * the gyro datasheet.
     *
     * @param voltsPerRadianPerSecond The sensitivity in volts/radian/second.
     */
    public void setSensitivity(double voltsPerRadianPerSecond) {
        this.voltsPerRadianPerSecond = voltsPerRadianPerSecond;
    }

    /**
     * Set the size of the neutral zone. Any voltage from the gyro less than
     * this amount from the center is considered stationary. Setting a deadband
     * will decrease the amount of drift when the gyro isn't rotating, but will
     * make it less accurate.
     *
     * @param volts The size of the deadband in volts
     */
    public void setDeadband(double volts) {
        int deadband = (int) (volts * 1e9 / analogInput.getLSBWeight() * (1 << analogInput.getOversampleBits()));
        analogInput.setAccumulatorDeadband(deadband);
    }

    /**
     * Sets the number of average bits. This causes 2^bits number of samples to
     * be averaged in the FPGA.
     * 
     * @param bits the number of average bits
     */
    public void setAverageBits(int bits) {
        analogInput.setAverageBits(bits);
        updateSampleRate();
    }

    /**
     * Gets the number of average bits.
     * 
     * @return the number of average bits
     */
    public int getAverageBits() {
        return analogInput.getAverageBits();
    }

    /**
     * Sets the number of oversample bits. This causes 2^bits number of samples
     * to be added together in the FPGA.
     * 
     * @param bits the number of oversample bits
     */
    public void setOversampleBits(int bits) {
        analogInput.setOversampleBits(bits);
        updateSampleRate();
    }

    /**
     * Gets the number of oversample bits.
     * 
     * @return the number of oversample bits
     */
    public int getOversampleBits() {
        return analogInput.getOversampleBits();
    }

    /**
     * Updates the FPGA sample rate if the average or oversample bits have
     * changed.
     */
    private void updateSampleRate() {
        double sr = sampleRate * (1 << (analogInput.getAverageBits() + analogInput.getOversampleBits()));
        AnalogInput.setGlobalSampleRate(sr);
    }

    /*
     * Live Window code, only does anything if live window is activated.
     */
    @Override
    public String getSmartDashboardType() {
        return "Gyro";
    }

    private ITable m_table;

    /**
     * {@inheritDoc}
     */
    @Override
    public void initTable(ITable subtable) {
        m_table = subtable;
        updateTable();
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public ITable getTable() {
        return m_table;
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void updateTable() {
        if (m_table != null) {
            m_table.putNumber("Value", getAngle());
        }
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void startLiveWindowMode() {
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void stopLiveWindowMode() {
    }
}
