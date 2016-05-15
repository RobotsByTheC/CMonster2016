/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016;

import org.usfirst.frc.team2084.CMonster2016.commands.AimArm;
import org.usfirst.frc.team2084.CMonster2016.commands.AimRobot;
import org.usfirst.frc.team2084.CMonster2016.commands.AimShot;
import org.usfirst.frc.team2084.CMonster2016.commands.BatterShot;
import org.usfirst.frc.team2084.CMonster2016.commands.ClearFaults;
import org.usfirst.frc.team2084.CMonster2016.commands.DriveHeading;
import org.usfirst.frc.team2084.CMonster2016.commands.HomeArm;
import org.usfirst.frc.team2084.CMonster2016.commands.Intake;
import org.usfirst.frc.team2084.CMonster2016.commands.LowGoalScore;
import org.usfirst.frc.team2084.CMonster2016.commands.PowerOffJetson;
import org.usfirst.frc.team2084.CMonster2016.commands.ResetArmAngle;
import org.usfirst.frc.team2084.CMonster2016.commands.ResetGyro;
import org.usfirst.frc.team2084.CMonster2016.commands.RotateToHeading;
import org.usfirst.frc.team2084.CMonster2016.commands.SetArmAngle;
import org.usfirst.frc.team2084.CMonster2016.commands.SetCameraAutoExposure;
import org.usfirst.frc.team2084.CMonster2016.commands.SetEncodersEnabled;
import org.usfirst.frc.team2084.CMonster2016.commands.SetFiringServo;
import org.usfirst.frc.team2084.CMonster2016.commands.SetIntakeCamera;
import org.usfirst.frc.team2084.CMonster2016.commands.SetIntakeSpeed;
import org.usfirst.frc.team2084.CMonster2016.commands.SetMeasuredArmAngle;
import org.usfirst.frc.team2084.CMonster2016.commands.SetShooterSpeed;
import org.usfirst.frc.team2084.CMonster2016.commands.TakeSnapshot;
import org.usfirst.frc.team2084.CMonster2016.commands.TankDrive;
import org.usfirst.frc.team2084.CMonster2016.commands.ToggleCamera;
import org.usfirst.frc.team2084.CMonster2016.subsystems.ArmSubsystem;
import org.usfirst.frc.team2084.CMonster2016.subsystems.IntakeSubsystem;
import org.usfirst.frc.team2084.CMonster2016.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVHatButton;
import edu.wpi.first.wpilibj.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public JoystickButton resetGyroButton;
    public JoystickButton trigger;
    public Joystick leftDriveJoystick;
    public Joystick rightDriveJoystick;
    public Joystick secondaryJoystick;
    public JoystickButton aimButton;
    public JoystickButton fireButton;
    public POVHatButton intakeButton;
    public JoystickButton expelButton;
    public JoystickButton portcullisButton;
    public JoystickButton snapshotButton;
    public POVHatButton lowGoalButton;
    public JoystickButton visionPrepareButton;
    public JoystickButton chevalDeFriseButton;
    public JoystickButton encoderEnableButton;
    public JoystickButton encoderDisableButton;
    public JoystickButton batterShotButton;
    public JoystickButton toggleCameraButton;

    public OI() {

        secondaryJoystick = new Joystick(2);

        leftDriveJoystick = new Joystick(0);
        rightDriveJoystick = new Joystick(1);

        resetGyroButton = new JoystickButton(leftDriveJoystick, 12);
        resetGyroButton.whenPressed(new ResetGyro());

        trigger = new JoystickButton(leftDriveJoystick, 1);
        trigger.whenPressed(
                new ParallelCommandGroup(new SetArmAngle(ArmSubsystem.COLLECT_ANGLE), new SetIntakeCamera(true)));
        trigger.whenReleased(new SetArmAngle(ArmSubsystem.DOWN_ANGLE));

        aimButton = new JoystickButton(secondaryJoystick, 8);
        aimButton.whileHeld(new AimShot());

        fireButton = new JoystickButton(secondaryJoystick, 6);
        fireButton.whenPressed(new SetFiringServo(true));
        fireButton.whenReleased(new SetFiringServo(false));

        intakeButton = new POVHatButton(leftDriveJoystick, 180);
        intakeButton.whileHeld(new Intake());

        expelButton = new JoystickButton(leftDriveJoystick, 5);
        expelButton.whileHeld(new SetIntakeSpeed(IntakeSubsystem.OUT_SPEED));

        lowGoalButton = new POVHatButton(leftDriveJoystick, 0);
        lowGoalButton.whileHeld(new LowGoalScore());
        lowGoalButton.whenReleased(new SetFiringServo(false));

        portcullisButton = new JoystickButton(leftDriveJoystick, 11);
        portcullisButton.whenPressed(new SetArmAngle(Math.toRadians(40)));

        snapshotButton = new JoystickButton(leftDriveJoystick, 9);
        snapshotButton.whenPressed(new TakeSnapshot());

        visionPrepareButton = new JoystickButton(secondaryJoystick, 1);
        visionPrepareButton.whenPressed(new ParallelCommandGroup(new SetArmAngle(ArmSubsystem.AIM_ANGLE),
                new SetShooterSpeed(ShooterSubsystem.INTAKE_SPEED)));

        chevalDeFriseButton = new JoystickButton(leftDriveJoystick, 6);
        chevalDeFriseButton.whenPressed(new SetArmAngle(Math.toRadians(20)));

        encoderEnableButton = new JoystickButton(leftDriveJoystick, 10);
        encoderEnableButton.whenPressed(new SetEncodersEnabled(true));

        encoderDisableButton = new JoystickButton(leftDriveJoystick, 9);
        encoderDisableButton.whenPressed(new SetEncodersEnabled(false));

        batterShotButton = new JoystickButton(secondaryJoystick, 7);
        batterShotButton.whileHeld(new BatterShot());

        toggleCameraButton = new JoystickButton(secondaryJoystick, 2);
        toggleCameraButton.whenPressed(new ToggleCamera());

        // SmartDashboard Buttons
        SmartDashboard.putData("Clear Faults", new ClearFaults());
        SmartDashboard.putData("Reset Gyro", new ResetGyro());
        SmartDashboard.putData("Set Arm Angle", new SetArmAngle(ArmSubsystem.COLLECT_ANGLE));
        SmartDashboard.putData("Aim Arm", new AimArm());
        SmartDashboard.putData("Aim Robot", new AimRobot());
        SmartDashboard.putData("Set Shooter Speed", new SetShooterSpeed(ShooterSubsystem.LOW_GOAL_SPEED));
        SmartDashboard.putData("Prepare Shot", new AimShot());
        SmartDashboard.putData("Intake", new Intake());
        SmartDashboard.putData("Home Arm", new HomeArm());
        SmartDashboard.putData("Reset Arm Position", new ResetArmAngle());
        SmartDashboard.putData("Set Firing Servo", new SetFiringServo(false));
        SmartDashboard.putData("Rotate To Heading", new RotateToHeading(0, 100));
        SmartDashboard.putData("Drive Heading", new DriveHeading(0, 0.4, 2));
        SmartDashboard.putData("Set Camera Auto Exposure", new SetCameraAutoExposure(true));
        SmartDashboard.putData("Take Snapshot", new TakeSnapshot());
        SmartDashboard.putData("Power Off Jetson", new PowerOffJetson());
        SmartDashboard.putData("Set Measured Arm Angle", new SetMeasuredArmAngle(Math.toRadians(65)));
        SmartDashboard.putData("Set Encoders Enabled", new SetEncodersEnabled(true));
        SmartDashboard.putData("Set Intake Camera", new SetIntakeCamera(false));
        SmartDashboard.putData("Tank Drive", new TankDrive());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    public Joystick getLeftDriveJoystick() {
        return leftDriveJoystick;
    }

    public Joystick getRightDriveJoystick() {
        return rightDriveJoystick;
    }

    public Joystick getSecondaryJoystick() {
        return secondaryJoystick;
    }

}
