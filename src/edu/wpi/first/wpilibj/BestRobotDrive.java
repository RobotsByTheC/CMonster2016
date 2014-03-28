package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2084.CMonster2014.RobotMap;

public class BestRobotDrive extends RobotDrive {

    public final double ROTATION_DEADBAND = 0.05;
    public static final double ROTATION_P = 0.01;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;
    public static final double ROTATION_F = 0.0;
    private double rotationSpeedPID = 0.0;
    private double gyroOffset = 0.0;

    private final PIDController rotationPIDController = new PIDController(
            ROTATION_P,
            ROTATION_I,
            ROTATION_D,
            ROTATION_F,
            RobotMap.driveSubsystemSteeringGyro,
            new PIDOutput() {
                public void pidWrite(double output) {
                    rotationSpeedPID = output;
                }
            }
    );

    public BestRobotDrive(final int frontLeftMotor, final int rearLeftMotor, final int frontRightMotor, final int rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        SmartDashboard.putData("Mecanum Drive Controller", rotationPIDController);
    }

    public BestRobotDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor, SpeedController frontRightMotor, SpeedController rearRightMotor) {
        super(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
        SmartDashboard.putData("Mecanum Drive Controller", rotationPIDController);
    }

    /**
     * Drive based on the specified joystick using the x and y axes.
     *
     * @param stick The joystick to use
     */
    public void mecanumDrive_Cartesian(GenericHID stick) {
        mecanumDrive_Cartesian(stick.getX(), stick.getY());
    }

    /**
     * Moves the robot forward and sideways at the specified speeds.
     *
     * @param x The forward speed (negative = backward, positive = forward)
     * @param y The sideways (crab) speed (negative = left, positive = right)
     *
     */
    public void mecanumDrive_Cartesian(double x, double y) {
        mecanumDrive_Cartesian(x, y, 0);
    }

    /**
     * Moves the robot forward and sideways at the specified speeds.
     *
     * @param x The forward speed (negative = backward, positive = forward)
     * @param y The sideways (crab) speed (negative = left, positive = right)
     * @param rotation The speed to rotate at while moving (negative =
     * clockwise, positive = counterclockwise)
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation) {
        mecanumDrive_Cartesian(x, y, rotation, RobotMap.driveSubsystemSteeringGyro.getAngle() - gyroOffset);

//        /**
//         * ******** our custom Mecanum drive (not necessary) ******
//         */
//        double fl = x + y + rotation;
//        double fr = y - x - rotation;
//        double rl = fr + rotation;
//        double rr = fl - rotation; 
//
//        double max = Math.max(Math.abs(fl), Math.abs(rl));
//        if (max > 1) {
//            fl /= max;
//            rl /= max;
//        }
//        setMotorOutputs(fl, rl, fr, rr);
    }

    /**
     * Drive method for Mecanum wheeled robots.
     *
     * A method for driving with Mecanum wheeled robots. There are 4 wheels
     * on the robot, arranged so that the front and back wheels are toed in 45 degrees.
     * When looking at the wheels from the top, the roller axles should form an X across the robot.
     *
     * This is designed to be directly driven by joystick axes.
     *
     * @param x The speed that the robot should drive in the X direction. [-1.0..1.0]
     * @param y The speed that the robot should drive in the Y direction.
     * This input is inverted to match the forward == -1.0 that joysticks produce. [-1.0..1.0]
     * @param rotation The rate of rotation for the robot that is completely independent of
     * the translation. [-1.0..1.0]
     * @param gyroAngle The current angle reading from the gyro.  Use this to implement field-oriented controls.
     */
    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
        rotation = getRotationPID(rotation);
        super.mecanumDrive_Cartesian(x, y, rotation, gyroAngle);
    }

    public void mecanumDrive_Orientation(double x, double y, double angle) {
        if (!rotationPIDController.isEnable() || rotationPIDController.getSetpoint() != angle) {
            rotationPIDController.setSetpoint(angle);
            rotationPIDController.enable();
        }

        super.mecanumDrive_Cartesian(x, y, -rotationSpeedPID, RobotMap.driveSubsystemSteeringGyro.getAngle());
    }

    private double getRotationPID(double rotationSpeed) {
        if (rotationPIDController.isEnable()) {
            if (Math.abs(rotationSpeed) >= ROTATION_DEADBAND) {
                rotationPIDController.disable();
            } else {
                return -rotationSpeedPID;
            }
        } else {
            if (Math.abs(rotationSpeed) < ROTATION_DEADBAND) {
                gyroOffset = RobotMap.driveSubsystemSteeringGyro.getAngle();
                rotationPIDController.setSetpoint(gyroOffset);
                rotationPIDController.enable();
            }
        }
        return rotationSpeed;
    }

    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
        rotateValue = getRotationPID(rotateValue);
        super.arcadeDrive(moveValue, -rotateValue, squaredInputs);
    }

    /**
     * Convenience method to set the motors to the specified speeds.
     *
     * @param frontleftOutput front left speed
     * @param rearLeftOutput rear left speed
     * @param frontRightOutput front right speed
     * @param rearRightOutput rear right speed
     */
    public void setMotorOutputs(double frontleftOutput, double rearLeftOutput, double frontRightOutput, double rearRightOutput) {
        if (m_frontLeftMotor == null || m_rearLeftMotor == null || m_frontRightMotor == null || m_rearRightMotor == null) {
            throw new NullPointerException("Null motor provided");
        }

        byte syncGroup = (byte) 0x80;

        m_frontLeftMotor.set(limit(frontleftOutput) * m_invertedMotors[MotorType.kFrontLeft_val] * m_maxOutput, syncGroup);
        m_rearLeftMotor.set(limit(rearLeftOutput) * m_invertedMotors[MotorType.kRearLeft_val] * m_maxOutput, syncGroup);

        m_frontRightMotor.set(limit(frontRightOutput) * m_invertedMotors[MotorType.kFrontRight_val] * m_maxOutput, syncGroup);
        m_rearRightMotor.set(limit(rearRightOutput) * m_invertedMotors[MotorType.kRearRight_val] * m_maxOutput, syncGroup);

        if (m_isCANInitialized) {
            try {
                CANJaguar.updateSyncGroup(syncGroup);
            } catch (CANNotInitializedException e) {
                m_isCANInitialized = false;
            } catch (CANTimeoutException e) {
            }
        }

        if (m_safetyHelper != null) {
            m_safetyHelper.feed();
        }
    }

    public void setLeftRightMotorOutputs(double leftOutput, double rightOutput) {
        if (m_rearLeftMotor == null || m_rearRightMotor == null) {
            throw new NullPointerException("Null motor provided");
        }

        byte syncGroup = (byte) 0x80;

        if (m_frontLeftMotor != null) {
            m_frontLeftMotor.set(limit(leftOutput) * m_invertedMotors[MotorType.kFrontLeft_val] * m_maxOutput, syncGroup);
        }
        m_rearLeftMotor.set(limit(leftOutput) * m_invertedMotors[MotorType.kRearLeft_val] * m_maxOutput, syncGroup);

        if (m_frontRightMotor != null) {
            m_frontRightMotor.set(limit(rightOutput) * m_invertedMotors[MotorType.kFrontRight_val] * m_maxOutput, syncGroup);
        }
        m_rearRightMotor.set(limit(rightOutput) * m_invertedMotors[MotorType.kRearRight_val] * m_maxOutput, syncGroup);

        if (m_isCANInitialized) {
            try {
                CANJaguar.updateSyncGroup(syncGroup);
            } catch (CANNotInitializedException e) {
                m_isCANInitialized = false;
            } catch (CANTimeoutException e) {
            }
        }

        if (m_safetyHelper != null) {
            m_safetyHelper.feed();
        }
    }

    /**
     * Moves the robot sideways at the specified speed.
     *
     * @param speed The speed and direction to crab (negative = left, positive =
     * right)
     */
    public void crab(double speed) {
        mecanumDrive_Cartesian(speed, 0);
    }

    public void resetGyro() {
        RobotMap.driveSubsystemSteeringGyro.reset();
        rotationPIDController.setSetpoint(0);
    }

    public void stopMotor() {
        super.stopMotor();
        if (m_safetyHelper != null) {
            m_safetyHelper.feed();
        }
    }
}
