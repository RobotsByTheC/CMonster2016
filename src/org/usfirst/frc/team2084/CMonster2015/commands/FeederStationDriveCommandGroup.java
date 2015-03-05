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

    public static final double LEFT_FEEDER_STATION_HEADING = Math.PI / 4;
    public static final double RIGHT_FEEDER_STATION_HEADING = -Math.PI / 4;
    public static final double LEFT_FEEDER_STATION_NOODLE_HEADING = (5 * Math.PI) / 4;
    public static final double RIGHT_FEEDER_STATION_NOODLE_HEADING = (3 * Math.PI) / 4;

    public FeederStationDriveCommandGroup(boolean left, boolean noodle) {
        addSequential(new RotateToCommand(left ? noodle ? LEFT_FEEDER_STATION_NOODLE_HEADING : LEFT_FEEDER_STATION_HEADING :
                noodle ? RIGHT_FEEDER_STATION_NOODLE_HEADING : RIGHT_FEEDER_STATION_HEADING, 0.1, 4.0));
    }
}
