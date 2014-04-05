package org.usfirst.frc2084.CMonster2014.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * DOES NOT WORK! DO NOT USE!
 */
public class SideAutonomousCommandGroup extends CommandGroup {

    public SideAutonomousCommandGroup(boolean leftGoal) {
        addSequential(new ResetGyroComand());
        addSequential(new SideAutonomousDriveCommand(leftGoal));
        addSequential(new FrontAutonomousDriveCommand(!leftGoal));
        addSequential(new SweeperEjectCommand());
    }
}
