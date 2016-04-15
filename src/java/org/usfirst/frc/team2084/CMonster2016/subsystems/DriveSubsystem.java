/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.subsystems;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.commands.ArcadeDrive;
import org.usfirst.frc.team2084.CMonster2016.drive.EncoderWheelController;
import org.usfirst.frc.team2084.CMonster2016.drive.GyroArcadeDriveAlgorithm;
import org.usfirst.frc.team2084.CMonster2016.drive.TwoWheelDriveController;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

/**
 * The subsystem that contains all the drive related components and sensors.
 * 
 * @author Ben Wolsieffer
 */
@SuppressWarnings("unused")
public class DriveSubsystem extends Subsystem {

    private final SpeedController leftTalon1 = RobotMap.driveSubsystemLeftTalon1;
    private final SpeedController leftTalon2 = RobotMap.driveSubsystemLeftTalon2;
    private final SpeedController rightTalon1 = RobotMap.driveSubsystemRightTalon1;
    private final SpeedController rightTalon2 = RobotMap.driveSubsystemRightTalon2;
    private final Encoder leftEncoder = RobotMap.driveSubsystemLeftEncoder;
    private final Encoder rightEncoder = RobotMap.driveSubsystemRightEncoder;

//    private final AnalogGyro gyro = RobotMap.driveSubsystemGyro;
    private final AHRS navX = RobotMap.driveSubsystemNavX;
    private final Accelerometer accelerometer = RobotMap.driveSubsystemAccelerometer;
    private final EncoderWheelController<SpeedController> leftWheels = RobotMap.driveSubsystemLeftWheels;
    private final EncoderWheelController<SpeedController> rightWheels = RobotMap.driveSubsystemRightWheels;
    private final TwoWheelDriveController<EncoderWheelController<SpeedController>> driveController =
            RobotMap.driveSubsystemDriveController;
    private final GyroArcadeDriveAlgorithm arcadeDriveAlgorithm = RobotMap.driveSubsystemArcadeDriveAlgorithm;

    public DriveSubsystem() {
        // Invert the right wheels
        rightWheels.setInverted(true);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDrive());
    }

    /**
     * Sets whether the encoders are enabled for driving during teleop.
     * 
     * @param enabled whether the encoders should be enabled
     */
    public void setEncodersEnabled(boolean enabled) {
        leftWheels.setEncoderEnabled(enabled);
        rightWheels.setEncoderEnabled(enabled);
    }
    
    public boolean getEncodersEnabled() {
        return leftWheels.isEncoderEnabled();
    }

    public void resetEncoders() {
        leftWheels.resetEncoder();
        rightWheels.resetEncoder();
    }

    public EncoderWheelController<SpeedController> getLeftWheels() {
        return leftWheels;
    }

    public EncoderWheelController<SpeedController> getRightWheels() {
        return rightWheels;
    }
}
