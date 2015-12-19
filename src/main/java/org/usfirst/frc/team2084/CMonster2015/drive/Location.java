/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.drive;

/**
 * Represents a location a certain distance from the robot's starting point.
 * 
 * @author Ben Wolsieffer
 */
public class Location {

    /**
     * The x coordinate of the location.
     */
    private final double x;
    /**
     * The y coordinate of the location.
     */
    private final double y;

    /**
     * Creates a new {@link Location} with the specified coordinates.
     * 
     * @param x the x coordinate
     * @param y the y coordinate
     */
    public Location(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Get the x coordinate of the location
     * 
     * @return the x coordinate
     */
    public double getX() {
        return x;
    }

    /**
     * Get the y coordinate of the location
     * 
     * @return the y coordinate
     */
    public double getY() {
        return y;
    }
}
