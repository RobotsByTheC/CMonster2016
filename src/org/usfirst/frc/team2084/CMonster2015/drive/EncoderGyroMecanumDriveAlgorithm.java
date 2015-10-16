/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

import org.usfirst.frc.team2084.CMonster2015.Gyro;

import edu.wpi.first.wpilibj.PIDController;

/**
 * A mecanum drive algorithm that uses a set of encoders and a gyro to do a lot
 * of closed loop control. It can drive the robot to a specific point and
 * orientation on the field, while also using all the gyro correction
 * capabilities of {@link GyroMecanumDriveAlgorithm}.
 * 
 * @author Ben Wolsieffer
 */
public class EncoderGyroMecanumDriveAlgorithm<S extends EncoderWheelController<?>> extends GyroMecanumDriveAlgorithm<S> {

    /**
     * The turning radius of the robot.
     */
    private final double turningRadius;

    /**
     * The PID controller that control movement in the x direction.
     */
    private final PIDController xLocationPIDController;
    /**
     * The PID controller that control movement in the y direction.
     */
    private final PIDController yLocationPIDController;

    /**
     * The output of the x PID controller.
     * 
     * @see EncoderGyroMecanumDriveAlgorithm#xLocationPIDController
     */
    private volatile double xPIDOutput;
    /**
     * The output of the y PID controller.
     * 
     * @see EncoderGyroMecanumDriveAlgorithm#yLocationPIDController
     */
    private volatile double yPIDOutput;

    /**
     * The cached location from the last update of the
     * {@link #xLocationPIDController}. This is an optimization to prevent the
     * constant recalculation of the location for both PID controllers.
     */
    private Location pidLocation = getLocation();

    /**
     * Creates a new {@link EncoderGyroMecanumDriveAlgorithm} using the
     * specified {@link FourWheelDriveController} and {@link Gyro}.
     * 
     * @param controller the drive controller with encoder wheel controllers
     * @param gyro the gyro to use
     */
    public EncoderGyroMecanumDriveAlgorithm(FourWheelDriveController<S> controller, Gyro gyro,
            PIDConstants headingPIDConstants, double headingTolerance,
            PIDConstants xLocationPIDConstants, PIDConstants yLocationPIDConstants,
            double xLocationTolerance, double yLocationTolerance,
            double driveBaseWidth, double driveBaseLength) {
        super(controller, gyro, headingPIDConstants, headingTolerance);

        // Calculate the turning radius of the robot; this is just the
        // pythagorean theorem.
        turningRadius = Math.sqrt(
                Math.pow(driveBaseWidth / 2, 2) + Math.pow(driveBaseLength / 2, 2));

        // Initialize the location PID controllers. They simply write their
        // outputs to variables.
        xLocationPIDController = DriveUtils.createPIDControllerFromConstants(xLocationPIDConstants,
                () -> (pidLocation = getLocation()).getX(), (o) -> xPIDOutput = o);
        xLocationPIDController.setAbsoluteTolerance(xLocationTolerance);
        yLocationPIDController = DriveUtils.createPIDControllerFromConstants(yLocationPIDConstants,
                pidLocation::getY, (o) -> yPIDOutput = o);
        yLocationPIDController.setAbsoluteTolerance(yLocationTolerance);
    }

    /**
     * Drives the robot to a location and heading. This only updates the speeds
     * of the motors once, so it needs to be called over and over until it
     * returns true (meaning the robot is on target).
     * 
     * @param location the location to drive to
     * @param heading the heading of the robot
     */
    public void driveToLocation(Location location, double heading) {
        driveToLocation(location, heading, 1.0, 1.0);
    }

    /**
     * Drives the robot to a location and heading. This only updates the speeds
     * of the motors once, so it needs to be called over and over until it
     * returns true (meaning the robot is on target). The movement and rotation
     * speeds are limited to the specified values.
     * 
     * @param location the location to drive to
     * @param heading the heading of the robot
     * @param maxMovementSpeed the maximum speed for the robot to move
     * @param maxRotationSpeed the maximum speed for the robot to rotate
     */
    public void driveToLocation(Location location, double heading, double maxMovementSpeed,
            double maxRotationSpeed) {
        xLocationPIDController.setSetpoint(location.getX());
        yLocationPIDController.setSetpoint(location.getY());
        if (!xLocationPIDController.isEnable() || !yLocationPIDController.isEnable()) {
            setLocationPIDControllerEnabled(true);
        }
        // Take a snapshot of the PID outputs
        double xPIDOutput = this.xPIDOutput;
        double yPIDOutput = this.yPIDOutput;

        // Get the signs of the PID outputs
        double xPIDOutputSign = xPIDOutput < 0 ? -1 : 1;
        double yPIDOutputSign = yPIDOutput < 0 ? -1 : 1;

        // Drive and limit the movement speed
        driveFieldHeadingCartesian(
                xPIDOutput > maxMovementSpeed ? maxMovementSpeed * xPIDOutputSign : xPIDOutput,
                yPIDOutput > maxMovementSpeed ? maxMovementSpeed * yPIDOutputSign : yPIDOutput,
                heading, maxRotationSpeed);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void driveCartesian(double x, double y) {
        // Disable location PID controller
        setLocationPIDControllerEnabled(false);
        super.driveCartesian(x, y);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void drivePolar(double magnitude, double direction, double rotation) {
        // Disable location PID controller
        setLocationPIDControllerEnabled(false);
        super.drivePolar(magnitude, direction, rotation);
    }

    /**
     * Gets the location of the robot relative to its starting position.
     * 
     * @return the robot location
     */
    public Location getLocation() {
        // Get the distance each wheel turned to achieve the current orientation
        double rotationDistance = getHeading() * turningRadius;

        // This might work...
        // Get distance each wheel has traveled accounting for the
        // rotationDistance
        double fl = controller.getFrontLeftWheel().getDistance() + rotationDistance;
        double fr = controller.getFrontRightWheel().getDistance() - rotationDistance;
        double rl = controller.getRearLeftWheel().getDistance() + rotationDistance;
        double rr = controller.getRearRightWheel().getDistance() - rotationDistance;

        // Average the distanced together (with the correct signs) to find the x
        // and y coordinates
        double x = (fl - fr - rl + rr) / 4;
        double y = (fl + fr + rl + rr) / 4;

        return new Location(x, y);
    }

    /**
     * Gets the distance of the robot from its desired location.
     * 
     * @return the error represented as a {@link Location}.
     */
    public Location getLocationError() {
        return new Location(xLocationPIDController.getError(), yLocationPIDController.getError());
    }

    public boolean isLocationOnTarget() {
        return xLocationPIDController.onTarget() && yLocationPIDController.onTarget();
    }

    /**
     * Convenience method to set whether the location PID controllers should be
     * enabled.
     * 
     * @param enabled whether the controllers should be enabled
     */
    private void setLocationPIDControllerEnabled(boolean enabled) {
        if (enabled) {
            // Only run if they aren't already enabled
            if (!xLocationPIDController.isEnable() || !yLocationPIDController.isEnable()) {
                // Reset PID output variables
                xPIDOutput = 0;
                yPIDOutput = 0;
                // Reset PID controllers
                xLocationPIDController.reset();
                yLocationPIDController.reset();
                // Enable PID controllers
                xLocationPIDController.enable();
                yLocationPIDController.enable();
            }
        } else {
            // Disable PID controllers
            xLocationPIDController.disable();
            yLocationPIDController.disable();
        }
    }
}
