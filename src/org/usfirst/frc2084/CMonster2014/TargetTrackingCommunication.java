
package org.usfirst.frc2084.CMonster2014;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 *
 * @author Ben Wolsieffer
 */
public class TargetTrackingCommunication {

    public static final String TARGET_TABLE_NAME = "TargetTracking";
    public static final String TARGET_TABLE_STATE_KEY = "goal_hot";
    public static final String TARGET_TABLE_AUTONOMOUS_VISION_RUNNING_KEY = "auto_vision";
    public static final String TARGET_TABLE_ENABLE_CAMERA_KEY = "enable_camera";

    public static final NetworkTable targetTable = NetworkTable.getTable(TARGET_TABLE_NAME);

    static {
        init();
    }
    
    public static void init() {
        setState(State.UNKNOWN);
        
    }
    
    public static class State {

        public static final int HOT_VALUE = 1;
        public static final int NOT_HOT_VALUE = 2;
        public static final int UNKNOWN_VALUE = 3;

        public final int value;

        public State(int value) {
            this.value = value;
        }

        public static final State HOT = new State(HOT_VALUE);
        public static final State NOT_HOT = new State(NOT_HOT_VALUE);
        public static final State UNKNOWN = new State(UNKNOWN_VALUE);

        public boolean equals(Object obj) {
            if (obj == null) {
                return false;
            }
            if (getClass() != obj.getClass()) {
                return false;
            }
            final State other = (State) obj;
            return this.value == other.value;
        }

        public int hashCode() {
            int hash = 3;
            hash = 29 * hash + this.value;
            return hash;
        }

        public String toString() {
            switch (value) {
                case HOT_VALUE:
                    return "HOT";
                case NOT_HOT_VALUE:
                    return "NOT HOT";
                case UNKNOWN_VALUE:
                default:
                    return "UNKNOWN";
            }
        }

    }

    public static void setState(State state) {
        targetTable.putNumber(TARGET_TABLE_STATE_KEY, state.value);
    }
    
    public static State getState() {
        return new State((int) targetTable.getNumber(TARGET_TABLE_STATE_KEY, State.UNKNOWN_VALUE));
    }

    public static boolean isAutonomousVisionRunning() {
        return targetTable.getBoolean(TARGET_TABLE_AUTONOMOUS_VISION_RUNNING_KEY, false);
    }

    public static void setAutonomousVisionRunning(boolean started) {
        targetTable.putBoolean(TARGET_TABLE_AUTONOMOUS_VISION_RUNNING_KEY, started);
    }

    public static void setCameraEnabled(boolean enabled) {
        targetTable.putBoolean(TARGET_TABLE_ENABLE_CAMERA_KEY, enabled);
    }
    
    public static boolean isCameraEnabled() {
        return targetTable.getBoolean(TARGET_TABLE_ENABLE_CAMERA_KEY, true);
    }
}
