/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.drive;

import java.lang.reflect.Field;

import org.usfirst.frc.team2084.CMonster2016.Gyro;
import org.usfirst.frc.team2084.CMonster2016.RollingAverage;
import org.usfirst.frc.team2084.CMonster2016.drive.processors.LinearRamper;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Ben Wolsieffer
 */
public class GyroArcadeDriveAlgorithm extends ArcadeDriveAlgorithm {

    public static final double DEFAULT_I_ZONE = 0.05;
    public static final double DEFAULT_TOLERANCE = 0.01;
    public static final int TOLERANCE_BUFFER_LENGTH = 20;
    public static final int PID_PERIOD = 10;
    public static final double DEFAULT_MIN_PID_OUTPUT = 0;
    public static final double DEFAULT_MAX_PID_OUTPUT = 0.6;
    public static final double PID_RAMP_RATE = 2.7;

    /**
     * The navX that the {@link GyroArcadeDriveAlgorithm} uses for rotating.
     */
    protected final AHRS gyro;

    /**
     * The output of the heading PID controller.
     */
    private volatile double headingPID = 0.0;

    /**
     * PID controller that maintains the orientation of the robot using the
     * gyro.
     */
    private final PIDController headingPIDController;

    private Field pidIAcculmulator;

    private double headingInverted = 1.0;
    private double tolerance = DEFAULT_TOLERANCE;

    private double iZone = DEFAULT_I_ZONE;
    private double maxPIDOutput = DEFAULT_MAX_PID_OUTPUT;
    private double minPIDOutput = DEFAULT_MIN_PID_OUTPUT;

    private final LinearRamper pidRamper = new LinearRamper(PID_RAMP_RATE, LinearRamper.Type.UP);

    private final RollingAverage averageError = new RollingAverage(50);

    /**
     * Creates a new {@link GyroArcadeDriveAlgorithm} using the parameters
     * described below.
     * 
     * @param controller the drive controller for this algorithm
     * @param gyro the gyroscope to use
     * @param headingPIDConstants the PID constants used to control the heading
     * @param headingTolerance the amount of error that is considered on target
     */
    public GyroArcadeDriveAlgorithm(DriveController<?> controller, AHRS gyro, PIDConstants headingPIDConstants) {
        super(controller);
        this.gyro = gyro;

        headingPIDController = DriveUtils.createPIDControllerFromConstants(headingPIDConstants, new PIDSource() {

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
            }

            @Override
            public double pidGet() {
                return getHeading();
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }
        }, (o) -> headingPID = -o, PID_PERIOD);

        setTolerance(DEFAULT_TOLERANCE);
        headingPIDController.setInputRange(-Math.PI, Math.PI);
        headingPIDController.setContinuous(true);
        setMaxPIDOutput(DEFAULT_MAX_PID_OUTPUT);
        headingPIDController.setToleranceBuffer(TOLERANCE_BUFFER_LENGTH);
        headingPIDController.disable();

        try {
            pidIAcculmulator = PIDController.class.getDeclaredField("m_totalError");
            pidIAcculmulator.setAccessible(true);
        } catch (NoSuchFieldException | SecurityException e) {
            e.printStackTrace();
        }

        SmartDashboard.putData("Heading PID Controller", headingPIDController);
    }

    public void driveHeading(double speed, double heading) {
        if (!headingPIDController.isEnabled()) {
            resetPID();
            headingPIDController.enable();
            pidRamper.reset();
        }
        if (Math.abs(headingPIDController.getError()) > iZone) {
            try {
                pidIAcculmulator.setDouble(headingPIDController, 0);
            } catch (IllegalArgumentException | IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        headingPIDController.setSetpoint(heading);

        averageError.newValue(Math.abs(headingPIDController.getError()));
        SmartDashboard.putNumber("Heading Avg. Error", Math.toDegrees(averageError.getAverage()));
        SmartDashboard.putNumber("Heading Error", Math.toDegrees(headingPIDController.getError()));

        // Add the minimum output to the PID output to get the real command.
        // This compensates for a dead band in the system
        double localHeadingPID = -headingPID;
        localHeadingPID += minPIDOutput * (localHeadingPID < 0 ? -1 : 1);

        arcadeDrive(speed, pidRamper.process(localHeadingPID));
    }

    public void rotateTo(double heading) {
        driveHeading(0, heading);
    }

    /**
     * Gets the heading of the robot in radians according to the gyro. This also
     * inverts the value if necessary. This *must* be used to retrieve the gyro
     * heading rather than calling {@link Gyro#getAngle()} to prevent race
     * conditions with the {@link PIDController}.
     * 
     * @return the heading
     */
    public double getHeading() {
        synchronized (this) {
            return Math.toRadians(gyro.getYaw()) * headingInverted;
        }
    }

    /**
     * Gets the rate of rotation of the robot in radians per second according to
     * the gyro. This also inverts the value if necessary. This *must* be used
     * to retrieve the rotation rate rather than calling {@link Gyro#getRate()}
     * to prevent race conditions with the {@link PIDController}.
     * 
     * @return the heading
     */
    public double getRotationRate() {
        synchronized (this) {
            return Math.toRadians(gyro.getRate() * headingInverted);
        }
    }

    /**
     * Gets whether the robot is facing the direction it should be. This always
     * returns true if the robot is being commanded to spin at a certain rate.
     * 
     * @return true if the robot is on target
     */
    public boolean isHeadingOnTarget() {
        if (headingPIDController.isEnabled()) {
            return Math.abs(averageError.getAverage()) < tolerance;
        } else {
            return true;
        }
    }

    public void setIZone(double iZone) {
        this.iZone = iZone;
    }

    public double getIZone() {
        return iZone;
    }

    public void setMinPIDOutput(double min) {
        minPIDOutput = min;
    }

    public double getMinPIDOutput() {
        return minPIDOutput;
    }

    public void setMaxPIDOutput(double max) {
        maxPIDOutput = max;
        headingPIDController.setOutputRange(-max, max);
    }

    public double getMaxPIDOutput() {
        return maxPIDOutput;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
        headingPIDController.setAbsoluteTolerance(tolerance);
    }

    public double getTolerance() {
        return tolerance;
    }

    public double getHeadingError() {
        return headingPIDController.getError();
    }

    public void resetPID() {
        headingPIDController.reset();
        averageError.reset(Math.abs(getHeadingError()));
        headingPID = 0;
    }

}
