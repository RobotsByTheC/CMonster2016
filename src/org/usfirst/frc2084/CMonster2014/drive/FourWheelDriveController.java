package org.usfirst.frc2084.CMonster2014.drive;

/**
 *
 * @author Ben Wolsieffer
 */
public class FourWheelDriveController extends DriveController {

    protected WheelController frontLeftWheel;
    protected WheelController frontRightWheel;
    protected WheelController rearLeftWheel;
    protected WheelController rearRightWheel;

    public FourWheelDriveController(WheelController frontLeftWheel, WheelController frontRightWheel, WheelController rearLeftWheel, WheelController rearRightWheel) {
        this.frontLeftWheel = frontLeftWheel;
        this.frontRightWheel = frontRightWheel;
        this.rearLeftWheel = rearLeftWheel;
        this.rearRightWheel = rearRightWheel;
    }

    public void drive(double leftSpeed, double rightSpeed) {
        frontLeftWheel.set(leftSpeed);
        rearLeftWheel.set(leftSpeed);
        frontRightWheel.set(rightSpeed);
        rearRightWheel.set(rightSpeed);

        safetyHelper.feed();
    }

    public void drive(double frontLeftSpeed, double frontRightSpeed, double rearLeftSpeed, double rearRightSpeed) {
        frontLeftWheel.set(frontLeftSpeed);
        frontRightWheel.set(frontRightSpeed);
        rearLeftWheel.set(rearLeftSpeed);
        rearRightWheel.set(rearRightSpeed);

        safetyHelper.feed();
    }

    public void stopMotor() {
        frontLeftWheel.set(0);
        rearLeftWheel.set(0);
        frontRightWheel.set(0);
        rearRightWheel.set(0);
    }

}
