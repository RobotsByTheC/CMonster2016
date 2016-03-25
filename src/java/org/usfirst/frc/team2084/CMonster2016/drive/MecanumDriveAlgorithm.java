/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.drive;

/**
 * A {@link DriveAlgorithm} that controls mecanum wheels without any sensors.
 * All the methods operate relative to the robot's current location.
 *
 * @author Ben Wolsieffer
 */
public class MecanumDriveAlgorithm<S extends WheelController<?>> extends DriveAlgorithm<FourWheelDriveController<S>> {

    /**
     * Creates a {@link MecanumDriveAlgorithm} usign the specified
     * {@link FourWheelDriveController}.
     * 
     * @param controller the drive controller
     */
    public MecanumDriveAlgorithm(FourWheelDriveController<S> controller) {
        super(controller);
    }

    /**
     * Moves the robot forward and sideways at the specified speeds. This moves
     * the robot relative to the robot's current orientation.
     *
     * @param x sideways (crab) speed (negative = left, positive = right)
     * @param y forward speed (negative = backward, positive = forward)
     */
    public void driveCartesian(double x, double y) {
        driveCartesian(x, y, 0);
    }

    /**
     * Moves the robot forward and sideways while rotating at the specified
     * speeds. This moves the robot relative to the robot's current orientation.
     *
     * @param x sideways (crab) speed (negative = left, positive = right)
     * @param y forward speed (negative = backward, positive = forward)
     * @param rotation The speed to rotate at while moving (negative =
     *        clockwise, positive = counterclockwise)
     */
    public void driveCartesian(double x, double y, double rotation) {
        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = x + y - rotation; // Front left speed
        wheelSpeeds[1] = -x + y + rotation; // Front right speed
        wheelSpeeds[2] = -x + y - rotation; // Rear left speed
        wheelSpeeds[3] = x + y + rotation; // Rear right speed

        DriveUtils.normalize(wheelSpeeds);

        controller.drive(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
    }

    /**
     * Drives the robot relative to itself with the specified magnitude,
     * direction and rotation.
     *
     * @param magnitude the speed that the robot should drive in a given
     *        direction.
     * @param direction the direction the robot should drive in radians,
     *        independent of rotation
     * @param rotation the rate of rotation for the robot that is completely
     *        independent of the magnitude or direction. [-1.0..1.0]
     */
    public void drivePolar(double magnitude, double direction, double rotation) {
        // Normalized for full power along the Cartesian axes.
        magnitude = DriveUtils.limit(magnitude) * Math.sqrt(2.0);
        // The rollers are at 45 degree (pi/4 radian) angles.
        direction += Math.PI / 4.0;
        double cosD = Math.cos(direction);
        double sinD = Math.sin(direction);

        double wheelSpeeds[] = new double[4];
        wheelSpeeds[0] = (sinD * magnitude + rotation);
        wheelSpeeds[1] = (cosD * magnitude - rotation);
        wheelSpeeds[2] = (cosD * magnitude + rotation);
        wheelSpeeds[3] = (sinD * magnitude - rotation);

        DriveUtils.normalize(wheelSpeeds);

        controller.drive(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
    }
}
