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

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Ben Wolsieffer
 */
public class GyroArcadeDriveAlgorithm extends ArcadeDriveAlgorithm {

    public static final double PID_I_ZONE = 0.05;
    public static final int TOLERANCE_BUFFER_LENGTH = 20;
    public static final int PID_PERIOD = 10;
    public static final double MAX_PID_OUTPUT = 0.6;
    public static final double PID_RAMP_RATE = 2.7;

    /**
     * The {@link Gyro} that the {@link GyroArcadeDriveAlgorithm} uses for
     * rotating.
     */
    protected final Gyro gyro;

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
    private double tolerance;

    private final LinearRamper pidRamper = new LinearRamper(PID_RAMP_RATE, LinearRamper.Type.UP);
    
    private final RollingAverage averageError = new RollingAverage(50);

    /**
     * @param driveController
     */
    public GyroArcadeDriveAlgorithm(DriveController<?> controller, Gyro gyro, PIDConstants headingPIDConstants, double headingTolerance) {
        super(controller);
        this.gyro = gyro;
        this.tolerance = headingTolerance;

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

        headingPIDController.setAbsoluteTolerance(headingTolerance);
        headingPIDController.setInputRange(-Math.PI, Math.PI);
        headingPIDController.setContinuous(true);
        headingPIDController.setOutputRange(-MAX_PID_OUTPUT, MAX_PID_OUTPUT);
        headingPIDController.setToleranceBuffer(TOLERANCE_BUFFER_LENGTH);

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
        if (Math.abs(headingPIDController.getError()) < PID_I_ZONE) {
            try {
                pidIAcculmulator.setDouble(headingPIDController, 0);
            } catch (IllegalArgumentException | IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        headingPIDController.setSetpoint(heading);

        averageError.newValue(Math.abs(headingPIDController.getError()));
        SmartDashboard.putNumber("heading avg err", averageError.getAverage());
        SmartDashboard.putNumber("heading err", headingPIDController.getError());
        
        // headingPIDController.setPID(headingPIDController.getP(),
        // Math.abs(headingPIDController.getError()) < 0.1 ?
        // headingPIDController.getI() : 0,
        // headingPIDController.getD());

        arcadeDrive(speed, pidRamper.process(-headingPID));
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
            return DriveUtils.normalizeHeading(gyro.getAngle() * headingInverted);
        }
    }

    /**
     * Sets the heading of the robot. This should be called rather than
     * {@link Gyro#setAngle(double)} to prevent the robot from trying to rotate
     * to this new heading, which is generally not the desired behavior.
     * 
     * @param heading
     */
    public void setHeading(double heading) {
        synchronized (this) {
            gyro.setAngle(DriveUtils.normalizeHeading(heading * headingInverted));
        }
    }

    /**
     * Gets the rate of rotation of the robot in radians per second according to
     * the gyro. This also inverts the value if necessary. This *must* be used
     * to retrieve the rotation rate rather than calling {@link Gyro#getRate(s)}
     * to prevent race conditions with the {@link PIDController}.
     * 
     * @return the heading
     */
    public double getRotationRate() {
        synchronized (this) {
            return gyro.getRate() * headingInverted;
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

    public double getHeadingError() {
        return headingPIDController.getError();
    }

    public void resetPID() {
        headingPIDController.reset();
        headingPID = 0;
    }

}
