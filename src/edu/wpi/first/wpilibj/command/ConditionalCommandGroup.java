/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.command;

/**
 * Command group that only runs its children if {@link #shouldRun()} returns
 * true when it is started.
 * 
 * @author Ben Wolsieffer
 */
public abstract class ConditionalCommandGroup extends CommandGroup {

    protected abstract boolean shouldRun();

    private boolean shouldRun = false;

    @Override
    void _initialize() {
        shouldRun = shouldRun();
        if (shouldRun) {
            super._initialize();
        }
    }

    @Override
    void _execute() {
        if (shouldRun) {
            super._execute();
        }
    }

    @Override
    protected boolean isFinished() {
        return !shouldRun || super.isFinished();
    }

    @Override
    void _interrupted() {
        if (shouldRun) {
            super._interrupted();
        }
    }
}
