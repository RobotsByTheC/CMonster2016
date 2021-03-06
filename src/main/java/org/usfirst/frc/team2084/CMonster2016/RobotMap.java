/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016;

import java.util.concurrent.Future;

import org.usfirst.frc.team2084.CMonster2016.drive.DIOEncoderWheelController;
import org.usfirst.frc.team2084.CMonster2016.drive.DriveController;
import org.usfirst.frc.team2084.CMonster2016.drive.EncoderWheelController;
import org.usfirst.frc.team2084.CMonster2016.drive.GyroArcadeDriveAlgorithm;
import org.usfirst.frc.team2084.CMonster2016.drive.TwoWheelDriveController;
import org.usfirst.frc.team2084.CMonster2016.drive.trajectory.TrajectoryGenerator;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;

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
    public static final double DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED = 3.35;

    /**
     * Wheel diameter in meters.
     */
    public static final double DRIVE_SUBSYSTEM_WHEEL_DIAMETER = 0.184;
    public static final double DRIVE_SUBSYSTEM_WHEELBASE_WIDTH = 0.5;

    public static final double DRIVE_SUBSYSTEM_BELT_RATIO = 42 / 39;

    public static final double DRIVE_SUBYSTEM_ENCODER_DISTANCE_PER_PULSE =
            (Math.PI * DRIVE_SUBSYSTEM_WHEEL_DIAMETER) / 2048 / DRIVE_SUBSYSTEM_BELT_RATIO;

    public static final double DRIVE_SUBSYSTEM_TRAJECTORY_PERIOD = 0.01;

    public static final double HALF_PI = Math.PI / 2;

    // Trajectories
    // Must be initialized outside of init() for unit tests to work
    public static final Trajectory.Config AUTONOMOUS_TRAJECTORY_CONFIG = new Config(Trajectory.FitMethod.HERMITE_CUBIC,
            Trajectory.Config.SAMPLES_FAST, DRIVE_SUBSYSTEM_TRAJECTORY_PERIOD, 1, 1, 20);
    public static final Waypoint[] CROSS_DEFENSE_WAYPOINTS = { new Waypoint(0, 0, 0), new Waypoint(5, 0, 0) };
    public static final Waypoint[] CHEVAL_APPROACH_WAYPOINTS = { new Waypoint(0, 0, 0), new Waypoint(1.75, 0, 0) };
    public static final Waypoint[] CHEVAL_CROSS_WAYPOINTS = { new Waypoint(0, 0, 0), new Waypoint(3, 0, 0) };
    public static final Waypoint[] POSITION_2_AUTONOMOUS_WAYPOINTS =
            { new Waypoint(0, 0, 0), new Waypoint(4.1, 0, 0), new Waypoint(4.4, -0.3, Math.toRadians(-45)) };
    public static final Waypoint[] POSITION_3_AUTONOMOUS_WAYPOINTS =
            { new Waypoint(0, 0, 0), new Waypoint(3.85, 0, 0), new Waypoint(4, -0.2, Math.toRadians(-15)) };
    public static final Waypoint[] POSITION_4_AUTONOMOUS_WAYPOINTS = { new Waypoint(0, 0, 0), new Waypoint(4.1, 0, 0) };
    public static final Waypoint[] POSITION_5_ROUGH_TERRAIN_WAYPOINTS =
            { new Waypoint(0, 0, 0), new Waypoint(3.8, 0, 0), new Waypoint(4.1, 1.4, Math.toRadians(15)) };
    public static final Waypoint[] POSITION_5_ROCK_WALL_WAYPOINTS =
            { new Waypoint(0, 0, 0), new Waypoint(4.3, 0, 0), new Waypoint(4.6, 1.4, Math.toRadians(15)) };

    public static final Future<Trajectory[]> CROSS_DEFENSE_TRAJECTORY =
            TrajectoryGenerator.generate(CROSS_DEFENSE_WAYPOINTS, AUTONOMOUS_TRAJECTORY_CONFIG);
    public static final Future<Trajectory[]> CHEVAL_APPROACH_TRAJECTORY =
            TrajectoryGenerator.generate(CHEVAL_APPROACH_WAYPOINTS, AUTONOMOUS_TRAJECTORY_CONFIG);
    public static final Future<Trajectory[]> CHEVAL_CROSS_TRAJECTORY =
            TrajectoryGenerator.generate(CHEVAL_CROSS_WAYPOINTS, AUTONOMOUS_TRAJECTORY_CONFIG);
    public static final Future<Trajectory[]> POSITION_2_AUTONOMOUS_TRAJECTORY =
            TrajectoryGenerator.generate(POSITION_2_AUTONOMOUS_WAYPOINTS, AUTONOMOUS_TRAJECTORY_CONFIG);
    public static final Future<Trajectory[]> POSITION_3_AUTONOMOUS_TRAJECTORY =
            TrajectoryGenerator.generate(POSITION_3_AUTONOMOUS_WAYPOINTS, AUTONOMOUS_TRAJECTORY_CONFIG);
    public static final Future<Trajectory[]> POSITION_4_AUTONOMOUS_TRAJECTORY =
            TrajectoryGenerator.generate(POSITION_4_AUTONOMOUS_WAYPOINTS, AUTONOMOUS_TRAJECTORY_CONFIG);
    public static final Future<Trajectory[]> POSITION_5_ROUGH_TERRAIN_TRAJECTORY =
            TrajectoryGenerator.generate(POSITION_5_ROUGH_TERRAIN_WAYPOINTS, AUTONOMOUS_TRAJECTORY_CONFIG);
    public static final Future<Trajectory[]> POSITION_5_ROCK_WALL_TRAJECTORY =
            TrajectoryGenerator.generate(POSITION_5_ROCK_WALL_WAYPOINTS, AUTONOMOUS_TRAJECTORY_CONFIG);

    public enum AutonomousPosition {
        POSITION_2,
        POSITION_3,
        POSITION_4,
        POSITION_5
    }

    public enum AutonomousDefense {
        ROCK_WALL,
        ROUGH_TERRAIN;
    }

    public enum AutonomousMode {
        POSITION_2_ROCK_WALL(POSITION_2_AUTONOMOUS_TRAJECTORY),
        POSITION_2_ROUGH_TERRAIN(POSITION_2_AUTONOMOUS_TRAJECTORY),
        POSITION_3_ROCK_WALL(POSITION_3_AUTONOMOUS_TRAJECTORY),
        POSITION_3_ROUGH_TERRAIN(POSITION_3_AUTONOMOUS_TRAJECTORY),
        POSITION_4_ROCK_WALL(POSITION_4_AUTONOMOUS_TRAJECTORY),
        POSITION_4_ROUGH_TERRAIN(POSITION_4_AUTONOMOUS_TRAJECTORY),
        POSITION_5_ROCK_WALL(POSITION_5_ROCK_WALL_TRAJECTORY),
        POSITION_5_ROUGH_TERRAIN(POSITION_5_ROUGH_TERRAIN_TRAJECTORY);

        public final Future<Trajectory[]> trajectory;

        private AutonomousMode(Future<Trajectory[]> trajectory) {
            this.trajectory = trajectory;
        }

        public static AutonomousMode get(AutonomousPosition position, AutonomousDefense defense) {
            return valueOf(position.name() + "_" + defense.name());
        }
    }

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
    public static Servo shooterSubsystemRightFiringServo;
    public static Servo shooterSubsystemLeftFiringServo;
    public static DigitalOutput shooterSubsystemJetsonPower;
    public static SpeedController intakeSubsystemSpark;

    /**
     * The analog gyro that was replaced by the navX.
     */
    // public static AnalogGyro driveSubsystemGyro;

    /**
     * The navX Micro used for autonomous and aiming
     */
    public static AHRS driveSubsystemNavX;

    /**
     * The builtin accelerometer in the roboRIO, not currently used for
     * anything.
     */
    public static Accelerometer driveSubsystemAccelerometer;

    // Generics are annoying
    @SuppressWarnings("rawtypes")
    public static ParameterBundle<DIOEncoderWheelController> driveSubsystemWheelParameters;

    /**
     * The front left wheel and encoder.
     */
    public static DIOEncoderWheelController<SpeedController> driveSubsystemLeftWheels;

    /**
     * The front right wheel and encoder.
     */
    public static DIOEncoderWheelController<SpeedController> driveSubsystemRightWheels;

    /**
     * The {@link DriveController} that links the four mecanum wheels.
     */
    public static TwoWheelDriveController<EncoderWheelController<SpeedController>> driveSubsystemDriveController;

    public static ParameterBundle<GyroArcadeDriveAlgorithm> driveSubsystemArcadeDriveAlgorithmParameters;
    /**
     * Drive algorithm
     */
    public static GyroArcadeDriveAlgorithm driveSubsystemArcadeDriveAlgorithm;
    // public static LEDController ledController;

    @SuppressWarnings("rawtypes")
    public static void init() {
        driveSubsystemLeftTalon1 = new Talon(0);
        LiveWindow.addActuator("Drive Subsystem", "Left Talon 1", (Talon) driveSubsystemLeftTalon1);

        driveSubsystemLeftTalon2 = new Talon(1);
        LiveWindow.addActuator("Drive Subsystem", "Left Talon 2", (Talon) driveSubsystemLeftTalon2);

        driveSubsystemRightTalon1 = new Talon(2);
        LiveWindow.addActuator("Drive Subsystem", "Right Talon 1", (Talon) driveSubsystemRightTalon1);

        driveSubsystemRightTalon2 = new Talon(3);
        LiveWindow.addActuator("Drive Subsystem", "Right Talon 2", (Talon) driveSubsystemRightTalon2);

        driveSubsystemLeftEncoder = new Encoder(19, 20, false, EncodingType.k4X);
        LiveWindow.addSensor("Drive Subsystem", "Left Encoder", driveSubsystemLeftEncoder);
        driveSubsystemLeftEncoder.setDistancePerPulse(DRIVE_SUBYSTEM_ENCODER_DISTANCE_PER_PULSE);
        driveSubsystemLeftEncoder.setPIDSourceType(PIDSourceType.kRate);
        driveSubsystemRightEncoder = new Encoder(22, 23, true, EncodingType.k4X);
        LiveWindow.addSensor("Drive Subsystem", "Right Encoder", driveSubsystemRightEncoder);
        driveSubsystemRightEncoder.setDistancePerPulse(DRIVE_SUBYSTEM_ENCODER_DISTANCE_PER_PULSE);
        driveSubsystemRightEncoder.setPIDSourceType(PIDSourceType.kRate);
        armSubsystemLeftTalon = new CANTalon(2);
        LiveWindow.addActuator("Arm Subsystem", "Left Talon", armSubsystemLeftTalon);

        armSubsystemRightTalon = new CANTalon(4);
        LiveWindow.addActuator("Arm Subsystem", "Right Talon", armSubsystemRightTalon);

        shooterSubsystemLeftTalon = new CANTalon(3);
        LiveWindow.addActuator("Shooter Subsystem", "Left Talon", shooterSubsystemLeftTalon);

        shooterSubsystemRightTalon = new CANTalon(1);
        LiveWindow.addActuator("Shooter Subsystem", "Right Talon", shooterSubsystemRightTalon);

        shooterSubsystemLeftFiringServo = new Servo(6);
        LiveWindow.addActuator("Shooter Subsystem", "Left Firing Servo", shooterSubsystemLeftFiringServo);

        shooterSubsystemRightFiringServo = new Servo(5);
        LiveWindow.addActuator("Shooter Subsystem", "Right Firing Servo", shooterSubsystemRightFiringServo);

        shooterSubsystemJetsonPower = new DigitalOutput(11);
        LiveWindow.addActuator("Shooter Subsystem", "Jetson Power", shooterSubsystemJetsonPower);

        intakeSubsystemSpark = new Spark(4);
        LiveWindow.addActuator("Intake Subsystem", "Spark", (Spark) intakeSubsystemSpark);

        // driveSubsystemGyro = new AnalogGyro(0);
        // LiveWindow.addSensor("Drive Subsystem", "Gyro", driveSubsystemGyro);

        driveSubsystemNavX = new AHRS(I2C.Port.kMXP, (byte) 100);
        LiveWindow.addSensor("Drive Subsystem", "navX", driveSubsystemNavX);

        driveSubsystemAccelerometer = new BuiltInAccelerometer(Range.k8G);
        LiveWindow.addSensor("Drive Subsystem", "Accelerometer", (BuiltInAccelerometer) driveSubsystemAccelerometer);

        driveSubsystemWheelParameters =
                new ParameterBundle<DIOEncoderWheelController>("Wheel Controller", DIOEncoderWheelController.class);

        driveSubsystemLeftWheels = new DIOEncoderWheelController<>(driveSubsystemLeftEncoder,
                driveSubsystemWheelParameters, "Parameters/left_wheel_pid_debug", DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED,
                new int[] { 2, 3 }, driveSubsystemLeftTalon1, driveSubsystemLeftTalon2);
        SmartDashboard.putData("Left Wheels", driveSubsystemLeftWheels);
        driveSubsystemRightWheels = new DIOEncoderWheelController<>(driveSubsystemRightEncoder,
                driveSubsystemWheelParameters, "Parameters/right_wheel_pid_debug", DRIVE_SUBSYSTEM_MAX_WHEEL_SPEED,
                new int[] { 13, 12 }, driveSubsystemRightTalon1, driveSubsystemRightTalon2);
        SmartDashboard.putData("Right Wheels", driveSubsystemRightWheels);

        driveSubsystemDriveController = new TwoWheelDriveController<EncoderWheelController<SpeedController>>(
                driveSubsystemLeftWheels, driveSubsystemRightWheels);

        driveSubsystemArcadeDriveAlgorithmParameters =
                new ParameterBundle<>("Arcade Drive Algorithm", GyroArcadeDriveAlgorithm.class);
        driveSubsystemArcadeDriveAlgorithm = new GyroArcadeDriveAlgorithm(driveSubsystemDriveController,
                driveSubsystemNavX, driveSubsystemArcadeDriveAlgorithmParameters);
        // ledController = new LEDController(Port.kUSB);
    }
}
