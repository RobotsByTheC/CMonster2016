package org.usfirst.frc2084.CMonster2014.commands;

import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2084.CMonster2014.Robot;
import org.usfirst.frc2084.CMonster2014.RobotMap;
import org.usfirst.frc2084.CMonster2014.TargetTrackingCommunication;

/**
 *
 */
public class FrontAutonomousDriveCommand extends Command {

    /**
     * The speed to drive at.
     */
    private static final double DRIVING_SPEED = 0.7;
    /**
     * The speed at which the robot should push itself against the wall to
     * maintain a straight path. Competition experience has shown that this is
     * not really necessary.
     */
    private static final double SIDEWAYS_SPEED = 0.2;
    /**
     * The rate at which the robot should accelerate.
     */
    private static final double ACCELERATION = 2;
    /**
     * The G-Force that the robot threshold at which the robot should stop. This
     * needs to be calibrated.
     */
    private static final double GFORCE_LIMIT = -0.7; //SET ME!!!!
    /**
     * The maximum time this part of autonomous can take before ending
     * automatically. This is to prevent a major problem should the robot not
     * detect when it hits the wall.
     */
    private static final double TIMEOUT = 5;
    /**
     * The amount of time the robot should wait from the beginning of the match
     * before ejecting the ball if the goal was not hot at the beginning of the
     * match.
     */
    private static final double WAIT_FOR_HOT_TIME = 6.0;
    /**
     * The amount of time the robot should continue driving after it hits the
     * wall in order to minimize bouncing.
     */
    private static final double WALL_PUSH_TIME = 0.4;
    private final boolean leftGoal;
    private double currentSpeed = 0.0;
    private boolean hitWall = false;
    private double hitWallTime = 0.0;

    public FrontAutonomousDriveCommand(boolean leftGoal) {
        super(TIMEOUT);
        this.leftGoal = leftGoal;
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.driveSubsystem);
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        currentSpeed = 0.0;
        hitWall = false;
        hitWallTime = 0.0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        if (currentSpeed < DRIVING_SPEED) {
            currentSpeed = timeSinceInitialized() * ACCELERATION;
        }
        double gForce = RobotMap.driveSubsystemAccelerometer.getAcceleration(ADXL345_I2C.Axes.kX);
        // If the gForce is greater than a limit, set the hit wall flag and 
        // record the time.
        if (GFORCE_LIMIT < 0 ? gForce < GFORCE_LIMIT : gForce > GFORCE_LIMIT) {
            hitWall = true;
            hitWallTime = timeSinceInitialized();
        }
        if (hitWall && (timeSinceInitialized() - hitWallTime > WALL_PUSH_TIME)) {
            Robot.driveSubsystem.getRobotDrive().stopMotor();
        } else {
            Robot.driveSubsystem.getRobotDrive().mecanumDrive_Orientation(leftGoal ? SIDEWAYS_SPEED : -SIDEWAYS_SPEED, currentSpeed, 0);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        // Was the goal not hot at the beginning of autonomous?
        boolean notHot = TargetTrackingCommunication.getState().equals(TargetTrackingCommunication.State.NOT_HOT);
        return (hitWall && (timeSinceInitialized() - hitWallTime > WALL_PUSH_TIME))
                && (notHot ? DriverStation.getInstance().getMatchTime() >= WAIT_FOR_HOT_TIME : true);
    }

    // Called once after isFinished returns true
    protected void end() {
        Robot.driveSubsystem.getRobotDrive().stopMotor();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
