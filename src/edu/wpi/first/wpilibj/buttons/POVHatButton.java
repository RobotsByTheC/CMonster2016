/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.buttons;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A "button" that is triggered by a certain direction of a POV hat.
 * 
 * @author Ben Wolsieffer
 */
public class POVHatButton extends Button {

    private final int direction;
    private final int hat;
    private final GenericHID joystick;

    /**
     * Creates a new POV hat button on the specified joystick, using the
     * specified hat and direction.
     * 
     * @param joystick the joystick to poll
     * @param hat the hat index to use
     * @param direction the direction to trigger on
     */
    public POVHatButton(GenericHID joystick, int hat, int direction) {
        this.direction = direction;
        this.hat = hat;
        this.joystick = joystick;
    }

    /**
     * 
     * 
     * @param joystick the joystick to poll
     * @param direction the direction to trigger on
     */
    public POVHatButton(GenericHID joystick, int direction) {
        this(joystick, 0, direction);
    }

    /**
     * @return true when the POV hat is pointing in the configured direction
     */
    @Override
    public boolean get() {
        return joystick.getPOV(hat) == direction;
    }

}
