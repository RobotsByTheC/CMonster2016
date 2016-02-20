/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.subsystems;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.RollingAverage;
import org.usfirst.frc.team2084.CMonster2016.commands.ManualShooterControl;
import org.usfirst.frc.team2084.CMonster2016.drive.PIDConstants;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.FeedbackDevice;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class ShooterSubsystem extends Subsystem {

   // @formatter:off
    public static final double[][] CALIBRATION_VALUES = { 
        // { distance (ft), angle (degrees), speed (RPM) }
        { 9, 25, 2500 }, 
        { 10, 30, 3000 },
        { 12, 35, 3000 }, 
        { 14, 40, 3000 }, 
        { 15, 42, 3200 }
    };
    //@formatter:on

    public static double getCalibrationAngle(double distance) {
        return Math.toRadians(calibrationInterpolate(distance, 0));
    }

    public static double getCalibrationSpeed(double distance) {
        return calibrationInterpolate(distance, 1);
    }

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

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS
    public static final double INTAKE_SPEED = -1000;
    public static final double LOW_GOAL_SPEED = 1000;
    public static final double FIRING_SERVO_EXTEND_POSITION = 0.26;
    public static final double FIRING_SERVO_RETRACT_POSITION = 0.685;
    public static final int ENCODER_CPR = 48;

    public static final double RAMP_RATE = 3;

    public static final double SPEED_ERROR_TOLERANCE = 100;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final CANTalon leftTalon = RobotMap.shooterSubsystemLeftTalon;
    private final CANTalon rightTalon = RobotMap.shooterSubsystemRightTalon;
    private final Servo firingServo = RobotMap.shooterSubsystemFiringServo;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private final RollingAverage averageError = new RollingAverage(10);

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

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
    }

    public void setShooterPower(double power) {
        leftTalon.changeControlMode(TalonControlMode.PercentVbus);
        rightTalon.changeControlMode(TalonControlMode.PercentVbus);

        rightTalon.set(-power);
        leftTalon.set(power);
    }

    public double getShooterSpeed() {
        return (leftTalon.getSpeed() + rightTalon.getSpeed()) / 2;
    }

    public void setShooterSpeed(double speed) {
        leftTalon.changeControlMode(TalonControlMode.Speed);
        rightTalon.changeControlMode(TalonControlMode.Speed);

        leftTalon.set(-speed);
        rightTalon.set(speed);

        averageError.newValue(Math.abs(speed - getShooterSpeed()));

    }

    public void stop() {
        setShooterPower(0);
    }

    public boolean onTarget() {
        return averageError.getAverage() < SPEED_ERROR_TOLERANCE;
    }

    public void resetAverageError() {
        averageError.reset();
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        setDefaultCommand(new ManualShooterControl());
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void setFiringServo(boolean fire) {
        firingServo.set(fire ? FIRING_SERVO_EXTEND_POSITION : FIRING_SERVO_RETRACT_POSITION);
    }
}
