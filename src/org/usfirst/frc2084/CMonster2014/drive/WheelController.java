package org.usfirst.frc2084.CMonster2014.drive;

import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author Ben Wolsieffer
 */
public class WheelController {

    protected SpeedController[] motors;
    private double inverted = 1;

    public WheelController(SpeedController motor) {
        this(new SpeedController[]{
            motor
        });
    }

    public WheelController(SpeedController motor, SpeedController motor2) {
        this(new SpeedController[]{
            motor,
            motor2
        });
    }

    public WheelController(SpeedController[] motors) {
        this.motors = motors;
    }

    public void set(double speed) {
        speed = DriveUtils.limit(speed) * inverted;
        for (int i = 0; i < motors.length; i++) {
            motors[i].set(speed);
        }
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted ? -1 : 1;
    }

    public boolean isInverted() {
        return inverted == -1;
    }
}
