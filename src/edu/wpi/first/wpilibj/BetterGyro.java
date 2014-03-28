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
