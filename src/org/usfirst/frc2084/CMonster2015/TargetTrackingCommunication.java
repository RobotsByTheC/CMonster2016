/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2015;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Class that manages communication with the vision tracking extension on the
 * SmartDashboard. This detects whether the target in front of the robot is hot
 * or not.
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

	/**
	 * Represents the state of the target: hot, not hot, or unknown.
	 */
	public static enum State {
		HOT, NOT_HOT, UNKNOWN;

		private static State[] values = values();
	}

	/**
	 * Set the state of the vision tracking system.
	 *
	 * @param state the state of the goal to set
	 */
	public static void setState(State state) {
		targetTable.putNumber(TARGET_TABLE_STATE_KEY, state.ordinal());
	}

	/**
	 * Get the state of the vision tracking system.
	 *
	 * @return the state of the goal
	 */
	public static State getState() {
		return State.values[(int) targetTable.getNumber(TARGET_TABLE_STATE_KEY, State.UNKNOWN.ordinal())];
	}

	/**
	 * Gets whether the system is trying to detect the target.
	 *
	 * @return true is the vision algorithm is running
	 */
	public static boolean isAutonomousVisionRunning() {
		return targetTable.getBoolean(TARGET_TABLE_AUTONOMOUS_VISION_RUNNING_KEY, false);
	}

	/**
	 * Sets whether the system should try to detect the target. This is
	 * different from {@link #setCameraEnabled(boolean)}, because it does not
	 * turn the stream on or off, but instead turn the processing algorithm on
	 * or off.
	 *
	 * @param started whether the algorithm should run
	 */
	public static void setAutonomousVisionRunning(boolean started) {
		targetTable.putBoolean(TARGET_TABLE_AUTONOMOUS_VISION_RUNNING_KEY, started);
	}

	/**
	 * Enable or disable the camera stream. The camera is disabled at the end of
	 * autonomous to save bandwidth.
	 *
	 * @param enabled whether to enable the camera stream
	 */
	public static void setCameraEnabled(boolean enabled) {
		targetTable.putBoolean(TARGET_TABLE_ENABLE_CAMERA_KEY, enabled);
	}

	/**
	 * Gets whether the camera stream is enabled.
	 *
	 * @return true if the camera is enabled
	 */
	public static boolean isCameraEnabled() {
		return targetTable.getBoolean(TARGET_TABLE_ENABLE_CAMERA_KEY, true);
	}
}
