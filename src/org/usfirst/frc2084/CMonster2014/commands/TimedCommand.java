package org.usfirst.frc2084.CMonster2014.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 * A command that does something once at the beginning but does not finish until
 * a certain time has elapsed.
 *
 * @author Ben Wolsieffer
 */
public abstract class TimedCommand extends Command {

    public TimedCommand(double time) {
        super(time);
    }

    /**
     * Initialize calls {@link #run()}.
     */
    protected final void initialize() {
        run();
    }

    protected abstract void run();

    protected final void execute() {
    }

    /**
     * This type of command never finishes but instead uses a timeout to end.
     *
     * @return false
     */
    protected boolean isFinished() {
        return false;
    }

}
