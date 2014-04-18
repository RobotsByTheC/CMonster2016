/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.usfirst.frc2084.CMonster2014.RobotMap;

/**
 *
 * @author Ben Wolsieffer
 */
public class MecanumDriveAlgorithm extends DriveAlgorithm {

    protected FourWheelDriveController controller;
    protected Gyro gyro;

    public final double ROTATION_DEADBAND = 0.05;
    public static final double ROTATION_P = 0.01;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final double ROTATION_F = 0.0;
    private double rotationSpeedPID = 0.0;
    private double gyroOffset = 0.0;

    private final PIDController rotationPIDController = new PIDController(
            ROTATION_P,
            ROTATION_I,
            ROTATION_D,
            ROTATION_F,
            gyro,
            new PIDOutput() {
                public void pidWrite(double output) {
                    rotationSpeedPID = output;
                }
            }
    );

    public MecanumDriveAlgorithm(FourWheelDriveController controller, Gyro gyro) {
        super(controller);
        this.controller = controller;
        this.gyro = gyro;
    }

    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
        rotation = getRotationPID(rotation);
        mecanumDrive_Cartesian0(x, y, rotation, gyroAngle);
    }

    /**
     * Moves the robot forward and sideways at the specified speeds.
     *
     * @param x The forward speed (negative = backward, positive = forward)
     * @param y The sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     * clockwise, positive = counterclockwise)
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation) {
        mecanumDrive_Cartesian(x, y, rotation, gyro.getAngle() - gyroOffset);
    }

    public void mecanumDrive_Orientation(double x, double y, double angle) {
        if (!rotationPIDController.isEnable() || rotationPIDController.getSetpoint() != angle) {
            rotationPIDController.setSetpoint(angle);
            rotationPIDController.enable();
        }

        mecanumDrive_Cartesian0(x, y, -rotationSpeedPID, gyro.getAngle());
    }

    /**
     * Drive method for Mecanum wheeled robots.
     *
     * A method for driving with Mecanum wheeled robots. There are 4 wheels on
     * the robot, arranged so that the front and back wheels are toed in 45
     * degrees. When looking at the wheels from the top, the roller axles should
     * form an X across the robot. This is very important.
     *
     * This is designed to be directly driven by joystick axes.
     *
     * @param x The speed that the robot should drive in the X direction.
     * [-1.0..1.0]
     * @param y The speed that the robot should drive in the Y direction. This
     * input is inverted to match the forward == -1.0 that joysticks produce.
     * [-1.0..1.0]
     * @param rotation The rate of rotation for the robot that is completely
     * independent of the translation. [-1.0..1.0]
     * @param gyroAngle The current angle reading from the gyro. Use this to
     * implement field-oriented controls.
     */
    private void mecanumDrive_Cartesian0(double x, double y, double rotation, double gyroAngle) {
        double xIn = x;
        double yIn = y;
        // Negate y for the joystick.
        yIn = -yIn;
        // Compenstate for gyro angle.
        double rotated[] = DriveUtils.rotateVector(xIn, yIn, gyroAngle);
        xIn = rotated[0];
        yIn = rotated[1];

        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = xIn + yIn + rotation;
        wheelSpeeds[1] = -xIn + yIn - rotation;
        wheelSpeeds[2] = -xIn + yIn + rotation;
        wheelSpeeds[3] = xIn + yIn - rotation;

        DriveUtils.normalize(wheelSpeeds);

        controller.drive(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
    }

    /**
     * Drive method for Mecanum wheeled robots.
     *
     * A method for driving with Mecanum wheeled robots. There are 4 wheels on
     * the robot, arranged so that the front and back wheels are toed in 45
     * degrees. When looking at the wheels from the top, the roller axles should
     * form an X across the robot.
     *
     * @param magnitude The speed that the robot should drive in a given
     * direction.
     * @param direction The direction the robot should drive in degrees. The
     * direction and magnitude are independent of the rotation rate.
     * @param rotation The rate of rotation for the robot that is completely
     * independent of the magnitude or direction. [-1.0..1.0]
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation) {
        // Normalized for full power along the Cartesian axes.
        magnitude = DriveUtils.limit(magnitude) * Math.sqrt(2.0);
        // The rollers are at 45 degree angles.
        double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = (sinD * magnitude + rotation);
        wheelSpeeds[1] = (cosD * magnitude - rotation);
        wheelSpeeds[2] = (cosD * magnitude + rotation);
        wheelSpeeds[3] = (sinD * magnitude - rotation);

        DriveUtils.normalize(wheelSpeeds);

        controller.drive(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
    }

    /**
     * Drive based on the specified joystick using the x and y axes.
     *
     * @param stick The joystick to use
     */
    public void mecanumDrive_Cartesian(GenericHID stick) {
        mecanumDrive_Cartesian(stick.getX(), stick.getY());
    }

    /**
     * Moves the robot forward and sideways at the specified speeds.
     *
     * @param x The forward speed (negative = backward, positive = forward)
     * @param y The sideways (crab) speed (negative = left, positive = right)
     *
     */
    public void mecanumDrive_Cartesian(double x, double y) {
        mecanumDrive_Cartesian(x, y, 0);
    }

    /**
     * Moves the robot sideways at the specified speed.
     *
     * @param speed The speed and direction to crab (negative = left, positive =
     * right)
     */
    public void crab(double speed) {
        mecanumDrive_Cartesian(speed, 0);
    }

    private double getRotationPID(double rotationSpeed) {
        if (rotationPIDController.isEnable()) {
            if (Math.abs(rotationSpeed) >= ROTATION_DEADBAND) {
                rotationPIDController.disable();
            } else {
                return -rotationSpeedPID;
            }
        } else {
            if (Math.abs(rotationSpeed) < ROTATION_DEADBAND) {
                gyroOffset = gyro.getAngle();
                rotationPIDController.setSetpoint(gyroOffset);
                rotationPIDController.enable();
            }
        }
        return rotationSpeed;
    }

    public void resetGyro() {
        gyro.reset();
        rotationPIDController.setSetpoint(0);
    }
}
