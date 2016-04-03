/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Autonomous mode that drives under the low bar and shoots using the vision
 * system.
 * 
 * @author Ben Wolsieffer
 */
public class CrossShootAutonomous extends CommandGroup {

    private static final SendableChooser defenseChooser = new SendableChooser();

    static {
        defenseChooser.addDefault("Rock Wall", CrossShootAutonomous.Defense.ROCK_WALL);
        defenseChooser.addObject("Rough Terrain", CrossShootAutonomous.Defense.ROUGH_TERRAIN);
        SmartDashboard.putData("Defense", defenseChooser);
    }

    public enum Defense {
        ROCK_WALL(3), ROUGH_TERRAIN(2.5);

        public final double time;

        private Defense(double time) {
            this.time = time;
        }
    }

    public CrossShootAutonomous(double time, double heading) {
        this(() -> time, heading);
    }

    public CrossShootAutonomous(double heading) {
        this(() -> ((Defense) defenseChooser.getSelected()).time, heading);
    }

    public CrossShootAutonomous(DoubleSupplier time, double heading) {
        addSequential(new CrossAutonomous(time));
        // Get the robot and arm into a position where the camera can see the
        // goal
        addSequential(new RotateToHeading(heading, true, 5));
        // Make sure the ball is out of the shooter wheels
        addParallel(new SetShooterSpeed(-1000));
        addSequential(new WaitCommand(0.5));
        addSequential(new AimAndFire());
        addSequential(new StopShooter());
    }

    /**
     * 
     */
    @Override
    protected void initialize() {
    }
}
