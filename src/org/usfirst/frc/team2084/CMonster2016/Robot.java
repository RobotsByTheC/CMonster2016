/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016;

import org.usfirst.frc.team2084.CMonster2016.commands.HomeArm;
import org.usfirst.frc.team2084.CMonster2016.commands.LoggingCommand;
import org.usfirst.frc.team2084.CMonster2016.commands.LowBarAutonomous;
import org.usfirst.frc.team2084.CMonster2016.commands.LowBarShootAutonomous;
import org.usfirst.frc.team2084.CMonster2016.commands.SetCameraAutoExposure;
import org.usfirst.frc.team2084.CMonster2016.subsystems.ArmSubsystem;
import org.usfirst.frc.team2084.CMonster2016.subsystems.DriveSubsystem;
import org.usfirst.frc.team2084.CMonster2016.subsystems.IntakeSubsystem;
import org.usfirst.frc.team2084.CMonster2016.subsystems.ShooterSubsystem;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the entry point for the robot application. Each method is
 * called at the appropriate time while the robot runs.
 * 
 * If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project root.
 */
public class Robot extends IterativeRobot {

    /**
     * The chooser on the SmartDashboard that we use to select our autonomous
     * mode.
     */
    private final SendableChooser autonomousChooser = new SendableChooser();

    /**
     * The currently selected autonomous {@link Command}, updated when
     * autonomous starts.
     */
    private Command autonomousCommand;

    /**
     * Always running {@link Command} that sends data to the SmartDashboard.
     */
    private Command loggingCommand;

    /**
     * Operator interface: defines all button, joysticks and other controls.
     */
    public static OI oi;

    /**
     * Global Power Distribution Panel (PDP) object.
     */
    public static PowerDistributionPanel pdp;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static DriveSubsystem driveSubsystem;
    public static ArmSubsystem armSubsystem;
    public static ShooterSubsystem shooterSubsystem;
    public static IntakeSubsystem intakeSubsystem;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * Called when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Initialize the robot map
        RobotMap.init();

        // Initialize subsystems
        driveSubsystem = new DriveSubsystem();
        armSubsystem = new ArmSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        intakeSubsystem = new IntakeSubsystem();

        // Initialize operator interface
        oi = new OI();
        pdp = new PowerDistributionPanel();

        // Start sending data to SmartDashboard
        loggingCommand = new LoggingCommand();
        loggingCommand.start();

        new SetCameraAutoExposure(false).start();

        // Add autonomous modes to the chooser
        autonomousChooser.addObject("Do nothing", null);
        autonomousChooser.addObject("Low Bar", new LowBarAutonomous());
        autonomousChooser.addObject("Low Bar and Shoot", new LowBarShootAutonomous());
        autonomousChooser.addDefault("Home Arm", new HomeArm());
        SmartDashboard.putData("Autonomous Mode", autonomousChooser);
        
    }

    /**
     * Called at the beginning of autonomous.
     */
    @Override
    public void autonomousInit() {
        Object autoMode = autonomousChooser.getSelected();
        if (autoMode instanceof Command) {
            autonomousCommand = (Command) autoMode;
            autonomousCommand.start();
        }
    }

    /**
     * Called periodically (~20ms) during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        // Run the command scheduler
        Scheduler.getInstance().run();
    }

    /**
     * Called at the beginning of teleop.
     */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * Called periodically (~20ms) during operator control.
     */
    @Override
    public void teleopPeriodic() {
        // Run the command scheduler
        Scheduler.getInstance().run();
    }

    /**
     * Called periodically (~20ms) during test mode.
     */
    @Override
    public void testPeriodic() {
        // Run the LiveWindow
        LiveWindow.run();
    }

    /**
     * Called when the robot is disabled.
     */
    @Override
    public void disabledInit() {
    }

    /**
     * Called periodically (~20ms) while the robot is disabled.
     */
    @Override
    public void disabledPeriodic() {
        // Run the command scheduler
        Scheduler.getInstance().run();
    }
}
