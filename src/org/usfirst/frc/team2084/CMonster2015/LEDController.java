/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

/**
 * Class for communicating with our Arduino LED controller over serial.
 * 
 * @author Ben Wolsieffer
 */
public class LEDController {

    /**
     * The type of pattern to display.
     */
    public static enum Pattern {
        DISABLE(0x3F),
        SOLID_RED(0x00),
        SOLID_BLUE(0x01),
        SOLID_GREEN(0x02),
        BLINK_RED(0x03),
        BLINK_BLUE(0x04),
        BLINK_GREEN(0x05),
        PULSE_RED(0x06),
        PULSE_BLUE(0x07),
        PULSE_GREEN(0x08),
        CONVERGE_RED(0x09),
        CONVERGE_BLUE(0x0A),
        CONVERGE_GREEN(0x0B),
        DIVERGE_RED(0x0C),
        DIVERGE_BLUE(0x0D),
        DIVERGE_GREEN(0x0E),
        RANDOM(0x0F);

        private final int code;

        private Pattern(int code) {
            this.code = code;
        }
    }

    /**
     * The {@link SerialPort} object used to communicate.
     */
    private final SerialPort serial;

    /**
     * Buffer that stores the pattern code.
     */
    private final ByteBuffer buffer = ByteBuffer.allocate(4);

    /**
     * Creates a new {@link LEDController} on the specified port.
     * 
     * @param port the serial port
     */
    public LEDController(Port port) {
        serial = new SerialPort(115200, port);
    }

    /**
     * Sends a pattern display command to the controller.
     * 
     * @see Pattern
     * @param pattern the pattern to display
     */
    public void setPattern(Pattern pattern) {
        serial.write(buffer.putInt(0, pattern.code).array(), 4);
    }
}
