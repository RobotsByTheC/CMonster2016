/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj;

/**
 *
 * @author Robot
 */
public class BetterGyro extends Gyro {

    public double resetOffset = 0.0;

    public BetterGyro(int slot, int channel) {
        super(slot, channel);
    }

    public BetterGyro(int channel) {
        super(channel);
    }

    public BetterGyro(AnalogChannel channel) {
        super(channel);
    }

    public double getAngle() {
        return super.getAngle() - resetOffset;
    }

    public void reset() {
        resetOffset = super.getAngle();
    }

}
