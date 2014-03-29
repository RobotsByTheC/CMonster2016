/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc2084.CMonster2014.commands;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 * @author ben
 */
public abstract class TimedCommand extends Command {

    public TimedCommand(double time) {
        super(time);
    }

    protected boolean isFinished() {
        return false;
    }

}
