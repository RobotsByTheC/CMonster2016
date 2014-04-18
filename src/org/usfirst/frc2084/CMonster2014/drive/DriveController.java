package org.usfirst.frc2084.CMonster2014.drive;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;

/**
 *
 * @author Ben Wolsieffer
 */
public abstract class DriveController implements MotorSafety {

    public static final double DEFAULT_SENSITIVITY = 0.5;
    public static final double DEFAULT_MAX_OUTPUT = 1.0;

    protected MotorSafetyHelper safetyHelper;

    private double sensitivity = DEFAULT_SENSITIVITY;

    private double maxOutput = DEFAULT_MAX_OUTPUT;

    public DriveController() {
        safetyHelper = new MotorSafetyHelper(this);
        safetyHelper.setSafetyEnabled(true);
    }

    public abstract void drive(double leftSpeed, double rightSpeed);

    public double getSensitivity() {
        return sensitivity;
    }

    public void setSensitivity(double sensitivity) {
        this.sensitivity = sensitivity;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public final void setExpiration(double expirationTime) {
        safetyHelper.setExpiration(expirationTime);
    }

    public final double getExpiration() {
        return safetyHelper.getExpiration();
    }

    public final boolean isAlive() {
        return safetyHelper.isAlive();
    }

    public abstract void stopMotor();

    public final void setSafetyEnabled(boolean enabled) {
        safetyHelper.setSafetyEnabled(enabled);
    }

    public final boolean isSafetyEnabled() {
        return safetyHelper.isSafetyEnabled();
    }

    public String getDescription() {
        return "Drive Controller";
    }
}
