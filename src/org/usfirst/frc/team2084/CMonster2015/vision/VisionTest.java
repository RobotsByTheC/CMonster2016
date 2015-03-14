/* 
 * Copyright (c) 2015 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2015.vision;

import java.io.IOException;

import org.opencv.core.Size;
import org.usfirst.frc.team2084.CMonster2015.vision.capture.CameraCapture;
import org.usfirst.frc.team2084.CMonster2015.vision.capture.CameraOpenException;

/**
 * A test for OpenCV that streams a camera image over the network. We didn't end
 * up using vision this year. :(
 * 
 * @author Ben Wolsieffer
 */
public class VisionTest {

    static {
        System.load("/usr/local/lib/lib_OpenCV/java/libopencv_java2410.so");
    }

    public static final int CAMERA_OPEN_ERROR = 1;
    public static final int VIDEO_SERVER_ERROR = 2;

    private final Size IMAGE_RESOLUTION = new Size(640, 480);
    private final CameraCapture camera = new CameraCapture(0);
    private final VisionProcessor processor = new ToteTrackingProcessor(camera);
    private VideoServer videoServer;

    /**
     * Runs the vision processing algorithm and sends the result over the
     * network.
     */
    public VisionTest() {
        try {
            videoServer = new VideoServer(8080, 75);
            videoServer.start();

            // Set up the camera.
            camera.setResolution(IMAGE_RESOLUTION);

            // Initialize the vision processor.
            processor.addImageHandler((image) -> {
                try {
                    videoServer.sendImage(image);
                } catch (IOException e) {
                    System.out.println("Cannot stream video over network: " + e);
                }
            });

            processor.start();

        } catch (IOException ioe) {
            System.out.println("Cannot start video server: " + ioe);
        } catch (CameraOpenException e) {
            System.out.println("Unable to open camera: " + e);
        }
    }
}
