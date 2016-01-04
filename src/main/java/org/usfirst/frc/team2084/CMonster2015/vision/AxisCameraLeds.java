/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.vision;

import java.io.IOException;
import java.net.InetAddress;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import net.schmizz.sshj.SSHClient;
import net.schmizz.sshj.connection.ConnectionException;
import net.schmizz.sshj.transport.TransportException;
import net.schmizz.sshj.transport.verification.PromiscuousVerifier;

/**
 * A class for interfacing with the status LEDs on an AXIS M1013 network camera.
 * It could be easily modified to work on other cameras as long as they use the
 * Linux sysfs LED driver and provide root access over SSH.
 * 
 * @author Ben Wolsieffer
 */
public class AxisCameraLeds {

    public class LED {

        /**
         * The sysfs directory that controls this LED
         */
        private final String directory;

        private final BrightnessProperty brightness;
        private final FileProperty<Trigger> trigger;

        private final FileProperty<Integer> delayOn;
        private final FileProperty<Integer> delayOff;

        /**
         * Create an LED with the specified name. This name must correspond to
         * the LED's sysfs name.
         * 
         * @param name the name of the LED
         */
        private LED(String name) {
            directory = "/sys/class/leds/" + name + "/";
            brightness = new BrightnessProperty(directory + "brightness");
            trigger = new FileProperty<>(directory + "trigger");

            delayOn = new FileProperty<>(directory + "delay_on");
            delayOff = new FileProperty<>(directory + "delay_off");
        }

        /**
         * Turns the LED on or off.
         * 
         * @param on true if the LED should be turned on
         */
        public void set(boolean on) {
            setTrigger(Trigger.NONE);
            brightness.set(on);
        }

        /**
         * Manually set the trigger of the LED. If you want to use
         * {@link Trigger#TIMER}, then you must use {@link #blink(int, int)}
         * instead of this method.
         * 
         * @param trigger the {@link Trigger} to set
         */
        public void setTrigger(Trigger trigger) {
            this.trigger.set(trigger);
        }

        /**
         * Blinks the LED with the specified on and off times.
         * 
         * @param onTime milliseconds the LED should be on each cycle
         * @param offTime milliseconds the LED should be off each cycle
         */
        public void blink(int onTime, int offTime) {
            setTrigger(Trigger.TIMER);
            delayOn.set(onTime);
            delayOff.set(offTime);
        }
    }

    /**
     * Represents the possible colors for a red/green LED.
     */
    public static enum Color {
        RED(true, false),
        GREEN(false, true),
        YELLOW(true, true),
        OFF(false, false);

        Color(boolean red, boolean green) {
            this.red = red;
            this.green = green;
        }

        private final boolean red;
        private final boolean green;
    }

    /**
     * Represents the possible triggers for an LED.
     */
    public static enum Trigger {
        NONE("none"),
        TIMER("timer"),
        INTERNAL_FLASH("nand-disk"),
        SD_CARD("mmc0"),
        NETWORK("network_green");

        private final String value;

        private Trigger(String value) {
            this.value = value;
        }

        @Override
        public String toString() {
            return value;
        }
    }

    private class BrightnessProperty extends FileProperty<Boolean> {

        public BrightnessProperty(String file) {
            super(file);
        }

        @Override
        protected String toStringValue(Boolean value) {
            return value ? "1" : "0";
        }

    }

    private class FileProperty<O> extends Property<O> {

        private final String file;

        private volatile String stringValue;

        public FileProperty(String file) {
            this.file = file;
        }

        protected String toStringValue(O value) {
            return value.toString();
        }

        @Override
        public void set(O value) {
            stringValue = toStringValue(value);
            super.set(value);
        }

        @Override
        protected void write(SSHClient client) throws ConnectionException, TransportException {
            String command = String.format("echo %s > %s", stringValue, file);
            System.out.println(command);
            client.startSession().exec(command).join();
        }
    }

    private abstract class Property<O> {

        protected volatile O value;

        public void set(O value) {
            if (value != this.value) {
                this.value = value;
                updateQueue.add(this);
            }
        }

        public O get() {
            return value;
        }

        protected abstract void write(
                SSHClient client) throws ConnectionException, TransportException;
    }

    private final InetAddress address;
    private final String password;

    private final LED leftRedStatusLed = new LED("status2:red");
    private final LED leftGreenStatusLed = new LED("status2:green");
    private final LED rightRedStatusLed = new LED("status:red");
    private final LED rightGreenStatusLed = new LED("status:green");

    private final BlockingQueue<Property<?>> updateQueue = new LinkedBlockingQueue<>();

    private class SSHThread implements Runnable {

        @Override
        public void run() {
            while (true) {
                try (SSHClient client = new SSHClient()) {
                    client.addHostKeyVerifier(new PromiscuousVerifier());
                    client.connect(address);
                    client.authPassword("root", password);

                    System.out.println("Connected.");

                    while (true) {
                        try {
                            Property<?> p = updateQueue.take();

                            if (p != null) {
                                p.write(client);
                            }
                        } catch (InterruptedException e) {
                        }
                    }
                } catch (IOException e) {
                    System.err.printf("Failed to connect to camera (%s) over SSH.\n", address);
                    e.printStackTrace();
                }
            }
        }

    }

    /**
     * 
     * @param address
     * @param password
     */
    public AxisCameraLeds(InetAddress address, String password) {
        this.address = address;
        this.password = password;

        Thread sshThread = new Thread(new SSHThread());
        sshThread.start();
    }

    /**
     * @return the left green status LED
     */
    public LED getLeftGreenStatusLed() {
        return leftGreenStatusLed;
    }

    /**
     * @return the left red status LED
     */
    public LED getLeftRedStatusLed() {
        return leftRedStatusLed;
    }

    /**
     * @return the right green status LED
     */
    public LED getRightGreenStatusLed() {
        return rightGreenStatusLed;
    }

    /**
     * @return the right red status LED
     */
    public LED getRightRedStatusLed() {
        return rightRedStatusLed;
    }

    /**
     * Sets the color of the left status LED.
     * 
     * @param color the color
     */
    public void setLeftStatusLedColor(Color color) {
        leftGreenStatusLed.set(color.green);
        leftRedStatusLed.set(color.red);
    }

    /**
     * Sets the color of the right status LED.
     * 
     * @param color the color
     */
    public void setRightStatusLedColor(Color color) {
        rightGreenStatusLed.set(color.green);
        rightRedStatusLed.set(color.red);
    }

    /**
     * Sets the color of the entire status ring.
     * 
     * @param color the color
     */
    public void setStatusLedColor(Color color) {
        setLeftStatusLedColor(color);
        setRightStatusLedColor(color);
    }
}
