/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive;

/**
 *
 * @author Ben Wolsieffer
 */
public abstract class DriveAlgorithm {

    protected DriveController controller;

    public DriveAlgorithm(DriveController controller) {
        this.controller = controller;
    }

    public void stop() {
        controller.stopMotor();
    }
}
