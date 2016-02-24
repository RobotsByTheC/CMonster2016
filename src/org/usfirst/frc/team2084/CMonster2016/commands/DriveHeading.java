/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.commands;

import java.util.function.DoubleSupplier;

import org.usfirst.frc.team2084.CMonster2016.RobotMap;
import org.usfirst.frc.team2084.CMonster2016.drive.processors.LinearRamper;
import org.usfirst.frc.team2084.CMonster2016.drive.processors.LinearRamper.Type;

/**
 * @author Ben Wolsieffer
 */
public class DriveHeading extends RotateToHeading {

    private static final String SPEED_KEY = "Speed";
    private static final String TIME_KEY = "Time";

    private LinearRamper speedRamper = new LinearRamper(0.5, Type.UP);

    private final DoubleSupplier speedSupplier;
    private double speed;
    private final DoubleSupplier timeSupplier;
    private double time;

    /**
     * @param heading
     */
    public DriveHeading(double heading, double speed, double time) {
        super(heading);

        addNumberParameter(SPEED_KEY, speed);
        speedSupplier = () -> getNumberParameter(SPEED_KEY);

        addNumberParameter(TIME_KEY, time);
        timeSupplier = () -> getNumberParameter(TIME_KEY);
    }

    /**
     * 
     */
    public DriveHeading(DoubleSupplier heading, DoubleSupplier speed, DoubleSupplier time) {
        super(heading);
        speedSupplier = speed;
        timeSupplier = time;
    }

    /**
     * 
     */
    @Override
    protected void initialize() {
        speedRamper.reset();
        
        speed = speedSupplier.getAsDouble();
        time = timeSupplier.getAsDouble();
        
        setTimeout(time);
        super.initialize();
    }

    /**
     * 
     */
    @Override
    protected void execute() {
        RobotMap.driveSubsystemArcadeDriveAlgorithm.driveHeading(speedRamper.process(speed), heading);
    }

    /**
     * @return
     */
    @Override
    protected boolean isFinished() {
        return isTimedOut();
    }

}
