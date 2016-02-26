/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016;

import org.usfirst.frc.team2084.CMonster2016.drive.DIOEncoderWheelController;
import org.usfirst.frc.team2084.CMonster2016.drive.DriveController;
import org.usfirst.frc.team2084.CMonster2016.drive.EncoderWheelController;
import org.usfirst.frc.team2084.CMonster2016.drive.GyroArcadeDriveAlgorithm;
import org.usfirst.frc.team2084.CMonster2016.drive.PIDConstants;
import org.usfirst.frc.team2084.CMonster2016.drive.TwoWheelDriveController;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    /**
     * The maximum speed the wheels can travel in meters/second.
     */
    public static final double DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED = 1.0;
    /**
     * Parameters that are used for the encoder based wheel speed PID
     * controller.
     */
    public static final PIDConstants DRIVE_SUBSYSTEM_WHEEL_SPEED_PID_CONSTANTS = new PIDConstants(0, 0, 0, 1);
    /**
     * Parameters that are used for the gyro heading PID controller.
     */
    public static final PIDConstants DRIVE_SUBSYSTEM_HEADING_PID_CONSTANTS = new PIDConstants(2.75, 0.1, 12);
    /**
     * Maximum allowed tolerance (in meters) that is considered on target for
     * location.
     */

    /**
     * Maximum allowed tolerance (in radians) that is considered on target for
     * heading.
     */
    public static final double DRIVE_SUBSYSTEM_HEADING_TOLERANCE = 0.01;

    public static SpeedController driveSubsystemLeftTalon1;
    public static SpeedController driveSubsystemLeftTalon2;
    public static SpeedController driveSubsystemRightTalon1;
    public static SpeedController driveSubsystemRightTalon2;
    public static Encoder driveSubsystemLeftEncoder;
    public static Encoder driveSubsystemRightEncoder;
    public static CANTalon armSubsystemLeftTalon;
    public static CANTalon armSubsystemRightTalon;
    public static CANTalon shooterSubsystemLeftTalon;
    public static CANTalon shooterSubsystemRightTalon;
    public static Servo shooterSubsystemFiringServo;
    public static DigitalOutput shooterSubsystemJetsonPower;
    public static SpeedController intakeSubsystemVictor;

    /**
     * The gyro used for autonomous and field oriented driving.
     */
    public static AnalogGyro driveSubsystemGyro;

    /**
     * The builtin accelerometer in the roboRIO, not currently used for
     * anything.
     */
    public static Accelerometer driveSubsystemAccelerometer;

    /**
     * The front left wheel and encoder.
     */
    public static EncoderWheelController<SpeedController> driveSubsystemLeftWheels;

    /**
     * The front right wheel and encoder.
     */
    public static EncoderWheelController<SpeedController> driveSubsystemRightWheels;

    /**
     * The {@link DriveController} that links the four mecanum wheels.
     */
    public static TwoWheelDriveController<EncoderWheelController<SpeedController>> driveSubsystemDriveController;

    /**
     * Drive algorithm
     */
    public static GyroArcadeDriveAlgorithm driveSubsystemArcadeDriveAlgorithm;
    // public static LEDController ledController;

    public static void init() {
        driveSubsystemLeftTalon1 = new Talon(0);
        LiveWindow.addActuator("Drive Subsystem", "Left Talon 1", (Talon) driveSubsystemLeftTalon1);

        driveSubsystemLeftTalon2 = new Talon(1);
        LiveWindow.addActuator("Drive Subsystem", "Left Talon 2", (Talon) driveSubsystemLeftTalon2);

        driveSubsystemRightTalon1 = new Talon(2);
        LiveWindow.addActuator("Drive Subsystem", "Right Talon 1", (Talon) driveSubsystemRightTalon1);

        driveSubsystemRightTalon2 = new Talon(3);
        LiveWindow.addActuator("Drive Subsystem", "Right Talon 2", (Talon) driveSubsystemRightTalon2);

        driveSubsystemLeftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive Subsystem", "Left Encoder", driveSubsystemLeftEncoder);
        driveSubsystemLeftEncoder.setDistancePerPulse(1.0);
        driveSubsystemLeftEncoder.setPIDSourceType(PIDSourceType.kRate);
        driveSubsystemRightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive Subsystem", "Right Encoder", driveSubsystemRightEncoder);
        driveSubsystemRightEncoder.setDistancePerPulse(1.0);
        driveSubsystemRightEncoder.setPIDSourceType(PIDSourceType.kRate);
        armSubsystemLeftTalon = new CANTalon(2);
        LiveWindow.addActuator("Arm Subsystem", "Left Talon", armSubsystemLeftTalon);

        armSubsystemRightTalon = new CANTalon(4);
        LiveWindow.addActuator("Arm Subsystem", "Right Talon", armSubsystemRightTalon);

        shooterSubsystemLeftTalon = new CANTalon(3);
        LiveWindow.addActuator("Shooter Subsystem", "Left Talon", shooterSubsystemLeftTalon);

        shooterSubsystemRightTalon = new CANTalon(1);
        LiveWindow.addActuator("Shooter Subsystem", "Right Talon", shooterSubsystemRightTalon);

        shooterSubsystemFiringServo = new Servo(5);
        LiveWindow.addActuator("Shooter Subsystem", "Firing Servo", shooterSubsystemFiringServo);

        shooterSubsystemJetsonPower = new DigitalOutput(11);
        LiveWindow.addActuator("Shooter Subsystem", "Jetson Power", shooterSubsystemJetsonPower);

        intakeSubsystemVictor = new Victor(4);
        LiveWindow.addActuator("Intake Subsystem", "Victor", (Victor) intakeSubsystemVictor);

        driveSubsystemGyro = new AnalogGyro(0);
        driveSubsystemAccelerometer = new BuiltInAccelerometer(Range.k8G);

        driveSubsystemLeftWheels =
                new DIOEncoderWheelController<>(driveSubsystemLeftEncoder, DRIVE_SUBSYSTEM_WHEEL_SPEED_PID_CONSTANTS, 1,
                        new int[] { 2, 3 }, driveSubsystemLeftTalon1, driveSubsystemLeftTalon2);
        driveSubsystemRightWheels =
                new DIOEncoderWheelController<>(driveSubsystemRightEncoder, DRIVE_SUBSYSTEM_WHEEL_SPEED_PID_CONSTANTS,
                        1, new int[] { 13, 12 }, driveSubsystemRightTalon1, driveSubsystemRightTalon2);

        driveSubsystemDriveController = new TwoWheelDriveController<EncoderWheelController<SpeedController>>(
                driveSubsystemLeftWheels, driveSubsystemRightWheels);

        driveSubsystemArcadeDriveAlgorithm = new GyroArcadeDriveAlgorithm(driveSubsystemDriveController,
                driveSubsystemGyro, DRIVE_SUBSYSTEM_HEADING_PID_CONSTANTS, DRIVE_SUBSYSTEM_HEADING_TOLERANCE);

        // ledController = new LEDController(Port.kUSB);
    }
}
