/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.Robot;
import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.drive.processors.InertiaGenerator;
import org.usfirst.frc.team2084.CMonster2016.drive.processors.RescalingDeadband;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Command that drives in arcade drive mode. This is the drive mode used by our
 * robot.
 *
 * @author Ben Wolsieffer
 */
@Parameter(key = ArcadeDrive.DEADBAND_KEY, type = Type.NUMBER, numberValue = ArcadeDrive.DEFAULT_DEADBAND)
@Parameter(key = ArcadeDrive.MAX_ROTATION_KEY, type = Type.NUMBER, numberValue = ArcadeDrive.DEFAULT_MAX_ROTATION)
public class ArcadeDrive extends Command {

    public static final String INERTIA_GAIN_KEY = "inertia_gain";
    public static final String DEADBAND_KEY = "deadband";
    public static final String MAX_ROTATION_KEY = "max_rotation";

    public static final double DEFAULT_INERTIA_GAIN = 0;
    public static final double DEFAULT_DEADBAND = 0.05;
    public static final double DEFAULT_MAX_ROTATION = 0.5;

    private final InertiaGenerator inertiaGenerator = new InertiaGenerator(DEFAULT_INERTIA_GAIN);
    private final RescalingDeadband deadband = new RescalingDeadband(DEFAULT_DEADBAND);

    private static final ParameterBundle<ArcadeDrive> parameters =
            new ParameterBundle<>("Arcade Drive Control", ArcadeDrive.class);

    public ArcadeDrive() {
        // This command drives, so it requires the drive subsystem.
        requires(Robot.driveSubsystem);

        parameters.addListener(DEADBAND_KEY, (key, type, value) -> {
            deadband.setDeadband((Double) value);
        });
    }

    @Override
    protected void initialize() {
        Robot.driveSubsystem.setEncodersEnabled(true);
        RobotMap.driveSubsystemLeftWheels.reset();
        RobotMap.driveSubsystemRightWheels.reset();
    }

    /**
     * Updates the robot speed and rotation based on the values of the drive
     * joystick.
     */
    @Override
    protected void execute() {
        Joystick j = Robot.oi.getLeftDriveJoystick();

        // Process the inputs
        double x = deadband.process(j.getX());
        double y = deadband.process(j.getY());
        // Scale the rotation down
        x *= x * (x < 0 ? -1 : 1) * parameters.getNumber(MAX_ROTATION_KEY);
        y *= y * (y < 0 ? -1 : 1);

        SmartDashboard.putNumber("Joystick X", x);
        SmartDashboard.putNumber("Joystick Y", y);
        RobotMap.driveSubsystemArcadeDriveAlgorithm.arcadeDrive(-y, inertiaGenerator.process(x));
    }

    /**
     * This command never ends on its own but it could be interrupted.
     *
     * @return false
     */
    @Override
    protected boolean isFinished() {
        return false;
    }

    /**
     * Stops the drive motors.
     */
    @Override
    protected void end() {
        RobotMap.driveSubsystemArcadeDriveAlgorithm.stop();
    }

    /**
     * Stops the drive motors.
     */
    @Override
    protected void interrupted() {
        end();
    }
}
