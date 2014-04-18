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
