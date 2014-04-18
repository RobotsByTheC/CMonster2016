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
