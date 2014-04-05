package org.usfirst.frc2084.CMonster2014.commands;

import org.usfirst.frc2084.CMonster2014.Robot;

/**
 * Command that extends the sweeper. It takes a certain amount of time for the
 * pneumatics to move so this command extends {@link TimedCommand} which keeps
 * it from ending immediately.
 *
 * @author Ben Wolsieffer
 */
public class SweeperExtendCommand extends TimedCommand {

    public SweeperExtendCommand() {
        // I estimated that it takes around 0.75 seconds to open the sweeper, so
        // that's how long this command takes.
        super(0.75);
        // This command uses the sweeper assembly so it requires the sweeper 
        // subsystem.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.sweeperSubsystem);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    /**
     * Extends the sweeper.
     */
    protected void run() {
        Robot.sweeperSubsystem.extend();
    }

    /**
     * Does nothing.
     */
    protected void end() {
    }

    /**
     * Does nothing.
     */
    protected void interrupted() {
    }
}
