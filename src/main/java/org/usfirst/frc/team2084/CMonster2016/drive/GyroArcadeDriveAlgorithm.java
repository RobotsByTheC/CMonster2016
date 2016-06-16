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
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author Ben Wolsieffer
 */
@Parameter(key = GyroArcadeDriveAlgorithm.P_KEY, type = Type.NUMBER, numberValue = GyroArcadeDriveAlgorithm.DEFAULT_P)
@Parameter(key = GyroArcadeDriveAlgorithm.I_KEY, type = Type.NUMBER, numberValue = GyroArcadeDriveAlgorithm.DEFAULT_I)
@Parameter(key = GyroArcadeDriveAlgorithm.D_KEY, type = Type.NUMBER, numberValue = GyroArcadeDriveAlgorithm.DEFAULT_D)
@Parameter(key = GyroArcadeDriveAlgorithm.I_ZONE_KEY, type = Type.NUMBER,
        numberValue = GyroArcadeDriveAlgorithm.DEFAULT_I_ZONE)
@Parameter(key = GyroArcadeDriveAlgorithm.RAMP_RATE_KEY, type = Type.NUMBER,
        numberValue = GyroArcadeDriveAlgorithm.DEFAULT_RAMP_RATE)
@Parameter(key = GyroArcadeDriveAlgorithm.HEADING_TOLERANCE_KEY, type = Type.NUMBER,
        numberValue = GyroArcadeDriveAlgorithm.DEFAULT_HEADING_TOLERANCE) // degrees
@Parameter(key = GyroArcadeDriveAlgorithm.MAX_PID_OUTPUT_KEY, type = Type.NUMBER,
        numberValue = GyroArcadeDriveAlgorithm.DEFAULT_MAX_PID_OUTPUT)
@Parameter(key = GyroArcadeDriveAlgorithm.MIN_PID_OUTPUT_KEY, type = Type.NUMBER,
        numberValue = GyroArcadeDriveAlgorithm.DEFAULT_MIN_PID_OUTPUT)
@Parameter(key = GyroArcadeDriveAlgorithm.DEBUG_KEY, type = Type.BOOLEAN, booleanValue = false)
public class GyroArcadeDriveAlgorithm extends ArcadeDriveAlgorithm {

    public static final String P_KEY = "p";
    public static final String I_KEY = "i";
    public static final String D_KEY = "d";
    public static final String I_ZONE_KEY = "i_zone";
    public static final String RAMP_RATE_KEY = "ramp_rate";
    public static final String HEADING_TOLERANCE_KEY = "heading_tolerance";
    public static final String MAX_PID_OUTPUT_KEY = "max_pid";
    public static final String MIN_PID_OUTPUT_KEY = "min_pid";
    public static final String DEBUG_KEY = "debug";

    public static final double DEFAULT_P = 0;
    public static final double DEFAULT_I = 0;
    public static final double DEFAULT_D = 0;
    public static final double DEFAULT_I_ZONE = 0.05;
    public static final double DEFAULT_HEADING_TOLERANCE = 0.01;
    public static final int TOLERANCE_BUFFER_LENGTH = 20;
    public static final double PID_PERIOD = 0.01;
    public static final double DEFAULT_MIN_PID_OUTPUT = 0;
    public static final double DEFAULT_MAX_PID_OUTPUT = 0.6;
    public static final double DEFAULT_RAMP_RATE = 2.7;

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

    private final ParameterBundle<GyroArcadeDriveAlgorithm> parameters;

    private final LinearRamper pidRamper = new LinearRamper(DEFAULT_RAMP_RATE, LinearRamper.Type.UP);

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
    public GyroArcadeDriveAlgorithm(DriveController<?> controller, AHRS gyro,
            ParameterBundle<GyroArcadeDriveAlgorithm> parameters) {
        super(controller);
        this.gyro = gyro;
        this.parameters = parameters;

        headingPIDController = new PIDController(0, 0, 0, new PIDSource() {

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

        parameters.addListener((key, type, val) -> {
            switch (key) {
            case P_KEY:
            case I_KEY:
            case D_KEY:
                double p = parameters.getNumber(P_KEY);
                double i = parameters.getNumber(I_KEY);
                double d = parameters.getNumber(D_KEY);
                headingPIDController.setPID(p, i, d);
            break;
            case HEADING_TOLERANCE_KEY:
                headingPIDController.setAbsoluteTolerance(Math.toRadians((Double) val));
            break;
            case MAX_PID_OUTPUT_KEY:
                double max = (Double) val;
                headingPIDController.setOutputRange(-max, max);
            break;
            }
        });
        headingPIDController.setInputRange(-Math.PI, Math.PI);
        headingPIDController.setContinuous(true);
        headingPIDController.setToleranceBuffer(TOLERANCE_BUFFER_LENGTH);
        headingPIDController.disable();

        try {
            pidIAcculmulator = PIDController.class.getDeclaredField("m_totalError");
            pidIAcculmulator.setAccessible(true);
        } catch (NoSuchFieldException | SecurityException e) {
            e.printStackTrace();
        }
    }

    private double[] debugPID = new double[3];

    public void driveHeading(double speed, double heading) {
        if (!headingPIDController.isEnabled()) {
            resetPID();
            headingPIDController.enable();
            pidRamper.reset();
        }
        if (Math.abs(headingPIDController.getError()) > getIZone()) {
            try {
                pidIAcculmulator.setDouble(headingPIDController, 0);
            } catch (IllegalArgumentException | IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (parameters.getBoolean(DEBUG_KEY)) {
            debugPID[0] = Timer.getFPGATimestamp();
            debugPID[1] = Math.toDegrees(DriveUtils.normalizeHeading(heading));
            debugPID[2] = Math.toDegrees(DriveUtils.normalizeHeading(getHeading()));
            parameters.getTable().putNumberArray("debug_pid", debugPID);
        }

        headingPIDController.setSetpoint(heading);

        averageError.newValue(Math.abs(headingPIDController.getError()));
        SmartDashboard.putNumber("Heading Avg. Error", Math.toDegrees(averageError.getAverage()));
        SmartDashboard.putNumber("Heading Error", Math.toDegrees(headingPIDController.getError()));

        // Add the minimum output to the PID output to get the real command.
        // This compensates for a dead band in the system
        double localHeadingPID = -headingPID;
        double minPIDOutput = getMinPIDOutput();
        if (Math.abs(localHeadingPID) < minPIDOutput) {
            localHeadingPID = minPIDOutput * (localHeadingPID < 0 ? -1 : 1);
        }

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
            return Math.abs(averageError.getAverage()) < getTolerance();
        } else {
            return true;
        }
    }

    /**
     * Gets the I zone of the PID controller. This serves as a way to prevent
     * integral windup.
     * 
     * @return
     */
    public double getIZone() {
        return Math.toRadians(parameters.getNumber(I_ZONE_KEY));
    }

    /**
     * Gets the minimum output of the PID controller. This serves as an efective
     * way to improve the response of the PID controller near the setpoint, as
     * it overcomes friction.
     * 
     * @return the minimum PID output
     */
    public double getMinPIDOutput() {
        return parameters.getNumber(MIN_PID_OUTPUT_KEY);
    }

    public void setTolerance(double tolerance) {
        parameters.setNumber(HEADING_TOLERANCE_KEY, Math.toDegrees(tolerance));
    }

    public double getTolerance() {
        return Math.toRadians(parameters.getNumber(HEADING_TOLERANCE_KEY));
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
