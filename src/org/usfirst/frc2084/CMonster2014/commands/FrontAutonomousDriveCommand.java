/* 
 * Copyright (c) 2014 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc2084.CMonster2014.commands;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2084.CMonster2014.Robot;
import org.usfirst.frc2084.CMonster2014.RobotMap;
import org.usfirst.frc2084.CMonster2014.TargetTrackingCommunication;

/**
 * Drives the robot forward until it hits the wall. It crabs left or right
 * slightly depending on which goal we are trying to score in. After the robot
 * hits the wall, it keeps driving (for {@link #WALL_PUSH_TIME} seconds) to
 * prevent it from bouncing off the wall slightly and missing the goal.
 */
public class FrontAutonomousDriveCommand extends Command {

    /**
     * The speed to drive towards the wall at. Setting this value to be higher
     * means that there will be more force on the robot and it would bounce
     * more. In the calibration of this value you need to make sure to account
     * for the fact that the robot goes slower when its rubbing against the wall
     * than it does when its being tested.
     */
    private static final double DRIVING_SPEED = 0.7;
    /**
     * The speed at which the robot should push itself against the wall to
     * maintain a straight path. Competition experience has shown that this is
     * not really necessary, but it really doesn't hurt much when it is set to a
     * small value.
     */
    private static final double SIDEWAYS_SPEED = 0.2;
    /**
     * The rate at which the robot should accelerate in motor-percent-power per
     * second. For example, a value of 1.0 for acceleration and 0.5 for speed
     * would mean that the robot would reach its max speed after 0.5 seconds.
     * This can probably be a pretty large value and is only necessary to keep
     * the robot from losing traction and possibly rotating.
     */
    private static final double ACCELERATION = 2.0;
    /**
     * The G-Force that the robot threshold at which the robot should stop. This
     * seems to be calibrated fine and has always worked (the robot hits the
     * wall pretty hard).
     */
    private static final double GFORCE_LIMIT = -0.7; //SET ME!!!!
    /**
     * The maximum time this part of autonomous can take before ending
     * automatically. This is to make sure the ball is expelled even if the
     * accelerometer is not working for some reason. Otherwise the robot would
     * just drive into the wall for the rest of autonomous, which would not be
     * good for a lot of reasons.
     */
    private static final double TIMEOUT = 5;
    /**
     * The amount of time the robot should wait from the beginning of the match
     * before ejecting the ball if the goal was not hot at the beginning of the
     * match. It is longer than the official 5 seconds just to be safe
     * (hopefully ball ejection doesn't need the full 5 seconds).
     */
    private static final double WAIT_FOR_HOT_TIME = 6.0;
    /**
     * The amount of time the robot should continue driving after it hits the
     * wall in order to minimize bouncing. The faster the robot travels, the
     * higher this value needs because the robot will bounce more.
     */
    private static final double WALL_PUSH_TIME = 0.4;
    /**
     * Stores whether this command was initialized to drive towards the left or
     * right walls. left = true, right = false
     */
    private final boolean leftGoal;
    /**
     * The speed the robot is currently set to travel at. This is updated each
     * time this command executes as it is running.
     */
    private double currentSpeed = 0.0;
    /**
     * Whether or not the robot has sensed that it hit the wall yet. It is
     * necessary to keep track of this so that {@link #WALL_PUSH_TIME} can work
     * after the robot hits the wall instead of just ending the command.
     */
    private boolean hitWall = false;
    /**
     * Records the time at which the robot hit the wall in seconds. This
     * subtracted from the current time is compared to {@link #WALL_PUSH_TIME}
     * to see if the robot has been pushing against the wall for long enough.
     */
    private double hitWallTime = 0.0;

    /**
     * Initializes this command to drive toward the left or right goal. This can
     * be done in the constructor because each time a command is run with a
     * certain configuration, a new instance is created.
     *
     * @param leftGoal drive toward the left goal (true) or right goal (false)
     * @see #leftGoal
     */
    public FrontAutonomousDriveCommand(boolean leftGoal) {
        // Setting a timeout makes sure the command ends even if it does not 
        // sense the deceleration of hitting the wall.
        super(TIMEOUT);
        this.leftGoal = leftGoal;
        // This command drives, therefore it requires the drive subsystem.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveSubsystem);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    /**
     * Makes sure all the instance variables are in their default states. Even
     * though autonomous is run only once each boot in competition, resetting
     * these values makes testing easier (so you don't have to reboot to test
     * autonomous each time).
     */
    protected void initialize() {
        currentSpeed = 0.0;
        hitWall = false;
        hitWallTime = 0.0;
    }

    /**
     * Each time this runs it updates the driving speed appropriately and checks
     * to see if the robot hit the wall yet.
     */
    protected void execute() {
        // If the robot has not yet reached its set driving speed, update its 
        // speed to match the acceleration rate.
        if (currentSpeed < DRIVING_SPEED) {
            // The cuurent speed is based on the absolute time since the command
            // started, multiplied by the acceleration factor.
            currentSpeed = timeSinceInitialized() * ACCELERATION;
        }
        // Get the current acceleration the robot is experiencing along the X 
        // axis.
        double gForce = RobotMap.driveSubsystemAccelerometer.getAcceleration(ADXL345_I2C.Axes.kX);
        // If the gForce is greater than the limit, set the hit wall flag and 
        // record the time. This tells the command that it should only run for 
        // WALL_PUSH_TIME more seconds. The if statement takes care of cases 
        // where the limit is both positive or negative because at the time I 
        // wrote this I did not know what direction the acceleration woud be in 
        // and it also eliminates a somewhat obscure bug that could occur if the
        // accelerometer was turned around.
        if (GFORCE_LIMIT < 0 ? gForce < GFORCE_LIMIT : gForce > GFORCE_LIMIT) {
            hitWall = true;
            hitWallTime = timeSinceInitialized();
        }
        // If the wall has already been hit and more than WALL_PUSH_TIME has 
        // passed since then, stop the motors. This allows the robot to stop 
        // even though it is stil waiting for the hot goal to change.
        if (hitWall && (timeSinceInitialized() - hitWallTime > WALL_PUSH_TIME)) {
            Robot.driveSubsystem.getRobotDrive().stopMotor();
        } else {
            // Otherwise drive forward and slightly sideways at the specified 
            // speeds. If the robot should drive towards the right wall, it 
            // inverts the sideways speed to make it do so. I used to use 
            // mecanumDrive_Cartesian, but it had an obscure bug that seemed to 
            // only appear in autonomous wher it would turn the robot slightly
            // when it started driving.
            Robot.driveSubsystem.getRobotDrive().mecanumDrive_Orientation(leftGoal ? SIDEWAYS_SPEED : -SIDEWAYS_SPEED, currentSpeed, 0);
        }
    }

    /**
     * Tells the scheduler to end the command when the robot has hit the wall,
     * pushed against it and wait the appropriate amount of time for the hot
     * goal to change (or not).
     *
     * @return whether to end the command
     */
    protected boolean isFinished() {
        // Was the goal not hot at the beginning of autonomous (the DS laptop 
        // tells the robot this)?
        boolean notHot = TargetTrackingCommunication.getState().equals(TargetTrackingCommunication.State.NOT_HOT);
        // The command finishes:
        // IF
        //   1. The robot has hit the wall and WALL_PUSH_TIME has elapsed since 
        //      it did so.
        // AND
        //   2. IF the goal was not hot at the beginning of autonomous:
        //        a. More than WAIT_FOR_HOT_TIME seconds have passed sine the 
        //           match started.
        //      ELSE (the goal was hot or unknown)
        //        b. Yes, it should finish. (always evaluates to true if the 
        //           goal has hot at the beginning).
        return (hitWall && (timeSinceInitialized() - hitWallTime > WALL_PUSH_TIME))
                && (notHot ? DriverStation.getInstance().getMatchTime() >= WAIT_FOR_HOT_TIME : true);
    }

    /**
     * Makes sure the robot stops driving when this command ends. The watchdog
     * timer in RobotDrive will do this automatically, but it is good to stop
     * the motors manually anyway.
     */
    protected void end() {
        Robot.driveSubsystem.getRobotDrive().stopMotor();
    }

    /**
     * This command should never be interrupted.
     */
    protected void interrupted() {
    }
}
