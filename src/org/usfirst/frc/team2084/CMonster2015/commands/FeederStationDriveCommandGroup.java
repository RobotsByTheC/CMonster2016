/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * {@link CommandGroup} that drives the robot nearer to the feeder station. This
 * is used if we are not getting a robot set.
 */
public class FeederStationDriveCommandGroup extends CommandGroup {

    /**
     * Heading the robot should face to line up with the left feeder station.
     */
    public static final double LEFT_FEEDER_STATION_HEADING = Math.PI / 4;
    /**
     * Heading the robot should face to line up with the right feeder station.
     */
    public static final double RIGHT_FEEDER_STATION_HEADING = -Math.PI / 4;
    /**
     * Heading the robot should face to line up with the left feeder station to
     * load a noodle.
     */
    public static final double LEFT_FEEDER_STATION_NOODLE_HEADING = (5 * Math.PI) / 4;
    /**
     * Heading the robot should face to line up with the left feeder station to
     * load a noodle.
     */
    public static final double RIGHT_FEEDER_STATION_NOODLE_HEADING = (3 * Math.PI) / 4;

    /**
     * 
     * Creates a new {@link FeederStationDriveCommandGroup} that drives to the
     * left or right and faces forward or backwards (to load a noodle).
     * 
     * @param left drive to the left feeder station
     * @param noodle get ready to load a noodle
     */
    public FeederStationDriveCommandGroup(boolean left, boolean noodle) {
        addSequential(new RotateToCommand(left ? noodle ? LEFT_FEEDER_STATION_NOODLE_HEADING : LEFT_FEEDER_STATION_HEADING :
                noodle ? RIGHT_FEEDER_STATION_NOODLE_HEADING : RIGHT_FEEDER_STATION_HEADING, 0.3, 4.0));
    }
}
