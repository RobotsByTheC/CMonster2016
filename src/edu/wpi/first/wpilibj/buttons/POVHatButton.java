/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package edu.wpi.first.wpilibj.buttons;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * @author ben
 */
public class POVHatButton extends Button {

    private final int direction;
    private final GenericHID joystick;

    /**
     * 
     */
    public POVHatButton(GenericHID joystick, int direction) {
        this.direction = direction;
        this.joystick = joystick;
    }

    /**
     * @return
     */
    @Override
    public boolean get() {
        return joystick.getPOV() == direction;
    }

}
