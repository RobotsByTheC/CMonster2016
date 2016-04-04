/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.subsystems;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.RollingAverage;
import org.usfirst.frc.team2084.CMonster2016.drive.PIDConstants;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Subsystem that controls our shooter.
 * 
 * @author Ben Wolsieffer
 */
public class ShooterSubsystem extends Subsystem {

   // @formatter:off
    /**
     * Interpolation table that is used to control the shooter.
     */
    public static final double[][] CALIBRATION_VALUES = { 
        // { distance (ft), angle (degrees), speed (RPM) }
        { 8.25,  45,   2500 },
        { 9.12,  43,   2700 },
        { 10.22, 42,   2700 },
        { 11.13, 39.5, 2700 },
        { 12.09, 38.5, 2700 },
        { 13.14, 38,   2700 },
        { 14.11, 37.5, 2700 }, 
        { 15.17, 37,   2700 },
        { 16.14, 36.5, 2700 }, 
        // stopped
        //{ 16,    37,   3000 }, 
        //{ 17,    35,   3000 }
    };
    //@formatter:on

    /**
     * Use the interpolation table to calculate the optimal shooting angle based
     * on the given distance to the goal.
     * 
     * @param distance the distance to the goal in feet
     * @return the shooting angle in radians
     */
    public static double getCalibrationAngle(double distance) {
        return Math.toRadians(calibrationInterpolate(distance, 0));
    }

    /**
     * Use the interpolation table to calculate the optimal shooting speed based
     * on the given distance to the goal.
     * 
     * @param distance the distance to the goal in feet
     * @return the shooting speed in RPM
     */
    public static double getCalibrationSpeed(double distance) {
        return calibrationInterpolate(distance, 1);
    }

    /**
     * Utility function that does the interpolation calculations for the
     * shooter.
     * 
     * @param distance the distance to the goal
     * @param column the column of the table to use
     * @return the interpolated value
     */
    private static double calibrationInterpolate(double distance, int column) {
        // Shift column to the right to be correct
        column++;

        if (distance <= CALIBRATION_VALUES[0][0]) {
            return CALIBRATION_VALUES[0][column];
        } else if (distance >= CALIBRATION_VALUES[CALIBRATION_VALUES.length - 1][0]) {
            return CALIBRATION_VALUES[CALIBRATION_VALUES.length - 1][column];
        } else {
            int r = 0;

            while (CALIBRATION_VALUES[r][0] < distance) {
                r++;
            }

            double[] lowCal = CALIBRATION_VALUES[r - 1];
            double[] highCal = CALIBRATION_VALUES[r];

            double slope = (highCal[column] - lowCal[column]) / (highCal[0] - lowCal[0]);
            return slope * (distance - lowCal[0]) + lowCal[column];
        }
    }

    public static final PIDConstants SHOOTER_PID_CONSTANTS = new PIDConstants(1.3, 0, 0, 0.69);

    public static final double INTAKE_SPEED = -2000;
    public static final double LOW_GOAL_SPEED = 2000;
    public static final double RIGHT_FIRING_SERVO_EXTEND_POSITION = 0.66;
    public static final double RIGHT_FIRING_SERVO_RETRACT_POSITION = 0.23;
    public static final double LEFT_FIRING_SERVO_EXTEND_POSITION = 0.075;
    public static final double LEFT_FIRING_SERVO_RETRACT_POSITION = 0.45;

    /**
     * Encoder counts per revolution.
     */
    public static final int ENCODER_CPR = 48;

    public static final double RAMP_RATE = 3;

    public static final double SPEED_ERROR_TOLERANCE = 100;

    private final CANTalon leftTalon = RobotMap.shooterSubsystemLeftTalon;
    private final CANTalon rightTalon = RobotMap.shooterSubsystemRightTalon;
    private final Servo leftFiringServo = RobotMap.shooterSubsystemLeftFiringServo;
    private final Servo rightFiringServo = RobotMap.shooterSubsystemRightFiringServo;

    /**
     * Rolling average of the shooter error.
     */
    private final RollingAverage averageError = new RollingAverage(5);

    /**
     * 
     */
    public ShooterSubsystem() {
        leftTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rightTalon.setFeedbackDevice(FeedbackDevice.QuadEncoder);

        leftTalon.configEncoderCodesPerRev(ENCODER_CPR);
        rightTalon.configEncoderCodesPerRev(ENCODER_CPR);

        ArmSubsystem.setTalonPID(leftTalon, SHOOTER_PID_CONSTANTS, 0, RAMP_RATE);
        ArmSubsystem.setTalonPID(rightTalon, SHOOTER_PID_CONSTANTS, 0, RAMP_RATE);

        setFiringServo(false);
    }

    /**
     * Sets the power of the shooter wheels.
     * 
     * @param power the power, between -1.0 and 1.0
     */
    public void setShooterPower(double power) {
        leftTalon.changeControlMode(TalonControlMode.PercentVbus);
        rightTalon.changeControlMode(TalonControlMode.PercentVbus);

        rightTalon.set(-power);
        leftTalon.set(power);
    }

    /**
     * Gets the average speed of the two shooter wheels.
     * 
     * @return the speed in RPM
     */
    public double getSpeed() {
        return (getLeftSpeed() + getRightSpeed()) / 2;
    }

    public double getLeftSpeed() {
        return leftTalon.getSpeed();
    }

    public double getRightSpeed() {
        return rightTalon.getSpeed();
    }

    /**
     * Sets the speed of the shooter wheels. This must be called repeatedly to
     * update the average error. Positive speeds shoot the ball, negative speeds
     * suck it in.
     * 
     * @param speed the speed in RPM
     */
    public void setShooterSpeed(double speed) {
        leftTalon.changeControlMode(TalonControlMode.Speed);
        rightTalon.changeControlMode(TalonControlMode.Speed);

        // Make sure power is never applied in the opposite direction from the
        // rotation.
        if (speed > 0) {
            leftTalon.configPeakOutputVoltage(0, -12);
            rightTalon.configPeakOutputVoltage(12, 0);
        } else if (speed < 0) {
            leftTalon.configPeakOutputVoltage(12, 0);
            rightTalon.configPeakOutputVoltage(0, -12);
        } else {
            // Edge case, if someone wants the wheels to stop, just let them
            // coast
            setShooterPower(0);
            return;
        }

        leftTalon.set(-speed);
        rightTalon.set(speed);

        averageError.newValue(Math.abs(speed - getSpeed()));

    }

    /**
     * Stops the shooter wheels.
     */
    public void stop() {
        setShooterPower(0);
    }

    /**
     * Gets whether the shooter wheel speed is on target.
     * 
     * @return true when the average speed is on target
     */
    public boolean onTarget() {
        return averageError.getAverage() < SPEED_ERROR_TOLERANCE;
    }

    /**
     * Reset the average error of the shooter speed.
     */
    public void resetAverageError() {
        averageError.reset();
    }

    @Override
    public void initDefaultCommand() {
    }

    /**
     * Sets the firing servo of the shooter.
     * 
     * @param fire if true, extend the servo arm
     */
    public void setFiringServo(boolean fire) {
        leftFiringServo.set(fire ? LEFT_FIRING_SERVO_EXTEND_POSITION : LEFT_FIRING_SERVO_RETRACT_POSITION);
        rightFiringServo.set(fire ? RIGHT_FIRING_SERVO_EXTEND_POSITION : RIGHT_FIRING_SERVO_RETRACT_POSITION);

    }
}
