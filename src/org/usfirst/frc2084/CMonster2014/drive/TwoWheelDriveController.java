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
public class TwoWheelDriveController extends DriveController {

    protected WheelController leftWheel;
    protected WheelController rightWheel;

    public TwoWheelDriveController(WheelController leftWheel, WheelController rightWheel) {
        this.leftWheel = leftWheel;
        this.rightWheel = rightWheel;
    }

    public void drive(double leftSpeed, double rightSpeed) {
        leftWheel.set(leftSpeed);
        rightWheel.set(rightSpeed);

        safetyHelper.feed();
    }

    public void stopMotor() {
        leftWheel.set(0);
        rightWheel.set(0);
    }

}
