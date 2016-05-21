/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * @author ben
 */
public class ChevalDeFriseCrossAutonomous extends CommandGroup {

    public ChevalDeFriseCrossAutonomous() {
        addSequential(new AutonomousSetup());
        addSequential(new ParallelCommandGroup(new PathFollower(RobotMap.CHEVAL_APPROACH_TRAJECTORY),
                new SetArmAngle(Math.toRadians(20))));
        addSequential(new SetArmAngle(0));
        addSequential(new ParallelCommandGroup(new PathFollower(RobotMap.CHEVAL_CROSS_TRAJECTORY),
                new SequentialCommandGroup(new WaitCommand(1.5), new SetArmAngle(Math.toRadians(20)))));
    }
}
