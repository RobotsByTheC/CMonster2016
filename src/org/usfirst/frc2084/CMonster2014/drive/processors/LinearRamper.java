/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.drive.processors;

import org.usfirst.frc2084.CMonster2014.Utils;

/**
 *
 * @author Ben Wolsieffer
 */
public class LinearRamper implements ValueProcessor {

    public static class Type {

        public static final int UP_VAL = 1;
        public static final int DOWN_VAL = 2;
        public static final int UP_DOWN_VAL = 3;

        public static final Type UP = new Type(UP_VAL);
        public static final Type DOWN = new Type(DOWN_VAL);
        public static final Type UP_DOWN = new Type(UP_DOWN_VAL);

        private final int val;

        private Type(int val) {
            this.val = val;
        }

        public int value() {
            return val;
        }
    }

    private final Type type;
    private final double rampRate;
    private double lastValue;
    private double lastTime;

    public LinearRamper(double rampRate, Type type) {
        this.rampRate = Math.abs(rampRate);
        this.type = type;
        reset();
    }

    public double process(double value) {
        // Calculate time since the ramper was last reset
        double elapsedTime = lastTime - Utils.getTime();
        double delta = value - lastValue;
        double maxDelta = rampRate / elapsedTime;
        double output = value;
        if (Math.abs(delta) > maxDelta) {
            delta = maxDelta * delta < 0 ? -1 : 1;
            output = lastValue + delta;
            if (delta < 0) {
                if (output < 0) {
                    if ((type.val | Type.UP_VAL) != Type.UP_VAL) {
                        output = value;
                    }
                } else {
                    if ((type.val | Type.DOWN_VAL) != Type.DOWN_VAL) {
                        output = value;
                    }
                }
            } else {
                if (output < 0) {
                    if ((type.val | Type.DOWN_VAL) != Type.DOWN_VAL) {
                        output = value;
                    }
                } else {
                    if ((type.val | Type.UP_VAL) != Type.UP_VAL) {
                        output = value;
                    }
                }
            }
        }
        lastValue = output;
        return output;
    }

    public void reset() {
        lastValue = 0;
        lastTime = Utils.getTime();
    }
}
