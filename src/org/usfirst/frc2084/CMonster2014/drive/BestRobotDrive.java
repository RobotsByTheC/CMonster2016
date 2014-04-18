package org.usfirst.frc2084.CMonster2014.drive;

import com.sun.squawk.util.Arrays;
import com.sun.squawk.util.MathUtils;
import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.can.CANNotInitializedException;
import edu.wpi.first.wpilibj.can.CANTimeoutException;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.parsing.IUtility;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc2084.CMonster2014.RobotMap;

/**
 * Utility class for handling Robot drive based on a definition of the motor
 * configuration. The robot drive class handles basic driving for a robot.
 * Currently, 4 and 8 motor standard drive trains are supported. Motor channel
 * numbers are passed supplied on creation of the class. Those are used for
 * either the drive function (intended for hand created drive code, such as
 * autonomous) or with the Tank/Arcade functions intended to be used for
 * Operator Control driving.
 */
public class BestRobotDrive implements MotorSafety, IUtility {

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

    protected MotorSafetyHelper m_safetyHelper;

    /**
     * The location of a motor on the robot for the purpose of driving
     */
    public static class MotorType {

        /**
         * The integer value representing this enumeration
         */
        public final int value;
        static final int kFrontLeft_val = 0;
        static final int kFrontRight_val = 1;
        static final int kRearLeft_val = 2;
        static final int kRearRight_val = 3;
        /**
         * motortype: front left
         */
        public static final MotorType kFrontLeft = new MotorType(kFrontLeft_val);
        /**
         * motortype: front right
         */
        public static final MotorType kFrontRight = new MotorType(kFrontRight_val);
        /**
         * motortype: rear left
         */
        public static final MotorType kRearLeft = new MotorType(kRearLeft_val);
        /**
         * motortype: rear right
         */
        public static final MotorType kRearRight = new MotorType(kRearRight_val);

        private MotorType(int value) {
            this.value = value;
        }
    }
    public static final double kDefaultExpirationTime = 0.1;
    public static final double kDefaultSensitivity = 0.5;
    public static final double kDefaultMaxOutput = 1.0;
    protected static final int kMaxNumberOfMotors = 8;
    protected final int m_invertedMotors[] = new int[kMaxNumberOfMotors];
    protected double m_sensitivity;
    protected double m_maxOutput;
    protected SpeedController m_frontLeftMotor = null;
    protected SpeedController m_frontLeftMotor2 = null;
    protected SpeedController m_frontRightMotor = null;
    protected SpeedController m_frontRightMotor2 = null;
    protected SpeedController m_rearLeftMotor = null;
    protected SpeedController m_rearLeftMotor2 = null;
    protected SpeedController m_rearRightMotor = null;
    protected SpeedController m_rearRightMotor2 = null;
    protected boolean m_allocatedSpeedControllers;
    protected boolean m_isCANInitialized = true;
    protected static boolean kArcadeRatioCurve_Reported = false;
    protected static boolean kTank_Reported = false;
    protected static boolean kArcadeStandard_Reported = false;
    protected static boolean kMecanumCartesian_Reported = false;
    protected static boolean kMecanumPolar_Reported = false;

    /**
     * Constructor for RobotDrive with 4 motors specified with channel numbers.
     * Set up parameters for a four wheel drive system where all four motor pwm
     * channels are specified in the call. This call assumes Jaguars for
     * controlling the motors.
     *
     * @param frontLeftMotor Front left motor channel number on the default
     * digital module
     * @param rearLeftMotor Rear Left motor channel number on the default
     * digital module
     * @param frontRightMotor Front right motor channel number on the default
     * digital module
     * @param rearRightMotor Rear Right motor channel number on the default
     * digital module
     */
    public BestRobotDrive(final int frontLeftMotor, final int rearLeftMotor,
            final int frontRightMotor, final int rearRightMotor) {
        this(new Jaguar(frontLeftMotor), new Jaguar(rearLeftMotor),
                new Jaguar(frontRightMotor), new Jaguar(rearRightMotor));
    }

    /**
     * Constructor for RobotDrive with 4 motors specified as
     * {@link SpeedController} objects.
     *
     * @param rearLeftMotor The back left SpeedController object used to drive
     * the robot.
     * @param frontLeftMotor The front left SpeedController object used to drive
     * the robot
     * @param rearRightMotor The back right SpeedController object used to drive
     * the robot.
     * @param frontRightMotor The front right SpeedController object used to
     * drive the robot.
     */
    public BestRobotDrive(SpeedController frontLeftMotor, SpeedController rearLeftMotor,
            SpeedController frontRightMotor, SpeedController rearRightMotor) {
        this(frontLeftMotor, null, rearLeftMotor, null,
                frontRightMotor, null, rearRightMotor, null);
    }

    /**
     * Constructor for RobotDrive with 8 motors specified with channel numbers.
     * Each wheel is driven with two motors, each controlled by a separate
     * {@link SpeedController}. This constructor assumes Jaguars for controlling
     * the motors.
     *
     * @param frontLeftMotor Front left motor channel number on the default
     * digital module
     * @param frontLeftMotor2 Second front left motor channel number on the
     * default digital module
     * @param rearLeftMotor Rear Left motor channel number on the default
     * digital module
     * @param rearLeftMotor2 Second rear Left motor channel number on the
     * default digital module
     * @param frontRightMotor Front right motor channel number on the default
     * digital module
     * @param frontRightMotor2 Second front right motor channel number on the
     * default digital module
     * @param rearRightMotor Rear Right motor channel number on the default
     * digital module
     * @param rearRightMotor2 Second rear Right motor channel number on the
     * default digital module
     */
    public BestRobotDrive(int frontLeftMotor, int frontLeftMotor2,
            int rearLeftMotor, int rearLeftMotor2,
            int frontRightMotor, int frontRightMotor2,
            int rearRightMotor, int rearRightMotor2) {
        this(new Jaguar(frontLeftMotor), new Jaguar(frontLeftMotor2),
                new Jaguar(rearLeftMotor), new Jaguar(rearLeftMotor2),
                new Jaguar(frontRightMotor), new Jaguar(frontRightMotor2),
                new Jaguar(rearRightMotor), new Jaguar(rearRightMotor2));
    }

    /**
     * Constructor for RobotDrive with 8 motors specified as SpeedController
     * objects. Each wheel is driven with two motors, each controlled by a
     * separate {@link SpeedController}.
     *
     * @param frontLeftMotor The front left SpeedController object used to drive
     * the robot
     * @param frontLeftMotor2 The second front left SpeedController object used
     * to drive the robot
     * @param rearLeftMotor The back left SpeedController object used to drive
     * the robot.
     * @param rearLeftMotor2 The second back left SpeedController object used to
     * drive the robot.
     * @param frontRightMotor The front right SpeedController object used to
     * drive the robot.
     * @param frontRightMotor2 The second front right SpeedController object
     * used to drive the robot.
     * @param rearRightMotor The back right SpeedController object used to drive
     * the robot.
     * @param rearRightMotor2 The second back right SpeedController object used
     * to drive the robot.
     */
    public BestRobotDrive(SpeedController frontLeftMotor, SpeedController frontLeftMotor2,
            SpeedController rearLeftMotor, SpeedController rearLeftMotor2,
            SpeedController frontRightMotor, SpeedController frontRightMotor2,
            SpeedController rearRightMotor, SpeedController rearRightMotor2) {
        if (frontLeftMotor == null || rearLeftMotor == null
                || frontRightMotor == null || rearRightMotor == null) {
            throw new NullPointerException("Null motor provided.");
        }
        if ((frontLeftMotor2 == null || rearLeftMotor2 == null
                || frontRightMotor2 == null || rearRightMotor2 == null)
                && (frontLeftMotor2 != null || rearLeftMotor2 != null
                || frontRightMotor2 != null || rearRightMotor2 != null)) {
            throw new NullPointerException("Some second motors are null.");
        }
        m_frontLeftMotor = frontLeftMotor;
        m_frontLeftMotor2 = frontLeftMotor2;
        m_rearLeftMotor = rearLeftMotor;
        m_rearLeftMotor2 = rearLeftMotor2;
        m_frontRightMotor = frontRightMotor;
        m_frontRightMotor2 = frontRightMotor2;
        m_rearRightMotor = rearRightMotor;
        m_rearRightMotor2 = rearRightMotor2;
        m_sensitivity = kDefaultSensitivity;
        m_maxOutput = kDefaultMaxOutput;
        Arrays.fill(m_invertedMotors, 1);
        m_allocatedSpeedControllers = false;
        setupMotorSafety();
        stopMotor();
        SmartDashboard.putData("Mecanum Drive Controller", rotationPIDController);
    }

    /**
     * Drive the motors at "speed" and "curve".
     *
     * The speed and curve are -1.0 to +1.0 values where 0.0 represents stopped
     * and not turning. The algorithm for adding in the direction attempts to
     * provide a constant turn radius for differing speeds.
     *
     * This function will most likely be used in an autonomous routine.
     *
     * @param outputMagnitude The forward component of the output magnitude to
     * send to the motors.
     * @param curve The rate of turn, constant for different forward speeds.
     */
    public void drive(double outputMagnitude, double curve) {
        double leftOutput, rightOutput;

        if (!kArcadeRatioCurve_Reported) {
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_ArcadeRatioCurve);
            kArcadeRatioCurve_Reported = true;
        }
        if (curve < 0) {
            double value = MathUtils.log(-curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude / ratio;
            rightOutput = outputMagnitude;
        } else if (curve > 0) {
            double value = MathUtils.log(curve);
            double ratio = (value - m_sensitivity) / (value + m_sensitivity);
            if (ratio == 0) {
                ratio = .0000000001;
            }
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude / ratio;
        } else {
            leftOutput = outputMagnitude;
            rightOutput = outputMagnitude;
        }
        setLeftRightMotorOutputs(leftOutput, rightOutput);
    }

    /**
     * Provide tank steering using the stored robot configuration. drive the
     * robot using two joystick inputs. The Y-axis will be selected from each
     * Joystick object.
     *
     * @param leftStick The joystick to control the left side of the robot.
     * @param rightStick The joystick to control the right side of the robot.
     */
    public void tankDrive(GenericHID leftStick, GenericHID rightStick) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getY(), rightStick.getY(), true);
    }

    /**
     * Provide tank steering using the stored robot configuration. drive the
     * robot using two joystick inputs. The Y-axis will be selected from each
     * Joystick object.
     *
     * @param leftStick The joystick to control the left side of the robot.
     * @param rightStick The joystick to control the right side of the robot.
     * @param squaredInputs Setting this parameter to true decreases the
     * sensitivity at lower speeds
     */
    public void tankDrive(GenericHID leftStick, GenericHID rightStick, boolean squaredInputs) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getY(), rightStick.getY(), squaredInputs);
    }

    /**
     * Provide tank steering using the stored robot configuration. This function
     * lets you pick the axis to be used on each Joystick object for the left
     * and right sides of the robot.
     *
     * @param leftStick The Joystick object to use for the left side of the
     * robot.
     * @param leftAxis The axis to select on the left side Joystick object.
     * @param rightStick The Joystick object to use for the right side of the
     * robot.
     * @param rightAxis The axis to select on the right side Joystick object.
     */
    public void tankDrive(GenericHID leftStick, final int leftAxis,
            GenericHID rightStick, final int rightAxis) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), true);
    }

    /**
     * Provide tank steering using the stored robot configuration. This function
     * lets you pick the axis to be used on each Joystick object for the left
     * and right sides of the robot.
     *
     * @param leftStick The Joystick object to use for the left side of the
     * robot.
     * @param leftAxis The axis to select on the left side Joystick object.
     * @param rightStick The Joystick object to use for the right side of the
     * robot.
     * @param rightAxis The axis to select on the right side Joystick object.
     * @param squaredInputs Setting this parameter to true decreases the
     * sensitivity at lower speeds
     */
    public void tankDrive(GenericHID leftStick, final int leftAxis,
            GenericHID rightStick, final int rightAxis, boolean squaredInputs) {
        if (leftStick == null || rightStick == null) {
            throw new NullPointerException("Null HID provided");
        }
        tankDrive(leftStick.getRawAxis(leftAxis), rightStick.getRawAxis(rightAxis), squaredInputs);
    }

    /**
     * Provide tank steering using the stored robot configuration. This function
     * lets you directly provide joystick values from any source.
     *
     * @param leftValue The value of the left stick.
     * @param rightValue The value of the right stick.
     * @param squaredInputs Setting this parameter to true decreases the
     * sensitivity at lower speeds
     */
    public void tankDrive(double leftValue, double rightValue, boolean squaredInputs) {

        if (!kTank_Reported) {
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_Tank);
            kTank_Reported = true;
        }

        // square the inputs (while preserving the sign) to increase fine control while permitting full power
        leftValue = limit(leftValue);
        rightValue = limit(rightValue);
        if (squaredInputs) {
            if (leftValue >= 0.0) {
                leftValue = (leftValue * leftValue);
            } else {
                leftValue = -(leftValue * leftValue);
            }
            if (rightValue >= 0.0) {
                rightValue = (rightValue * rightValue);
            } else {
                rightValue = -(rightValue * rightValue);
            }
        }
        setLeftRightMotorOutputs(leftValue, rightValue);
    }

    /**
     * Provide tank steering using the stored robot configuration. This function
     * lets you directly provide joystick values from any source.
     *
     * @param leftValue The value of the left stick.
     * @param rightValue The value of the right stick.
     */
    public void tankDrive(double leftValue, double rightValue) {
        tankDrive(leftValue, rightValue, true);
    }

    /**
     * Arcade drive implements single stick driving. Given a single Joystick,
     * the class assumes the Y axis for the move value and the X axis for the
     * rotate value. (Should add more information here regarding the way that
     * arcade drive works.)
     *
     * @param stick The joystick to use for Arcade single-stick driving. The
     * Y-axis will be selected for forwards/backwards and the X-axis will be
     * selected for rotation rate.
     * @param squaredInputs If true, the sensitivity will be decreased for small
     * values
     */
    public void arcadeDrive(GenericHID stick, boolean squaredInputs) {
        // simply call the full-featured arcadeDrive with the appropriate values
        arcadeDrive(stick.getY(), stick.getX(), squaredInputs);
    }

    /**
     * Arcade drive implements single stick driving. Given a single Joystick,
     * the class assumes the Y axis for the move value and the X axis for the
     * rotate value. (Should add more information here regarding the way that
     * arcade drive works.)
     *
     * @param stick The joystick to use for Arcade single-stick driving. The
     * Y-axis will be selected for forwards/backwards and the X-axis will be
     * selected for rotation rate.
     */
    public void arcadeDrive(GenericHID stick) {
        this.arcadeDrive(stick, true);
    }

    /**
     * Arcade drive implements single stick driving. Given two joystick
     * instances and two axis, compute the values to send to either two or four
     * motors.
     *
     * @param moveStick The Joystick object that represents the forward/backward
     * direction
     * @param moveAxis The axis on the moveStick object to use for
     * forwards/backwards (typically Y_AXIS)
     * @param rotateStick The Joystick object that represents the rotation value
     * @param rotateAxis The axis on the rotation object to use for the rotate
     * right/left (typically X_AXIS)
     * @param squaredInputs Setting this parameter to true decreases the
     * sensitivity at lower speeds
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis,
            boolean squaredInputs) {
        double moveValue = moveStick.getRawAxis(moveAxis);
        double rotateValue = rotateStick.getRawAxis(rotateAxis);

        arcadeDrive(moveValue, rotateValue, squaredInputs);
    }

    /**
     * Arcade drive implements single stick driving. Given two joystick
     * instances and two axis, compute the values to send to either two or four
     * motors.
     *
     * @param moveStick The Joystick object that represents the forward/backward
     * direction
     * @param moveAxis The axis on the moveStick object to use for
     * forwards/backwards (typically Y_AXIS)
     * @param rotateStick The Joystick object that represents the rotation value
     * @param rotateAxis The axis on the rotation object to use for the rotate
     * right/left (typically X_AXIS)
     */
    public void arcadeDrive(GenericHID moveStick, final int moveAxis,
            GenericHID rotateStick, final int rotateAxis) {
        this.arcadeDrive(moveStick, moveAxis, rotateStick, rotateAxis, true);
    }

    /**
     * Arcade drive implements single stick driving. This function lets you
     * directly provide joystick values from any source.
     *
     * @param moveValue The value to use for forwards/backwards
     * @param rotateValue The value to use for the rotate right/left
     * @param squaredInputs If set, decreases the sensitivity at low speeds
     */
    public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
        rotateValue = -getRotationPID(rotateValue);
        // local variables to hold the computed PWM values for the motors
        if (!kArcadeStandard_Reported) {
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_ArcadeStandard);
            kArcadeStandard_Reported = true;
        }

        double leftMotorSpeed;
        double rightMotorSpeed;

        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        setLeftRightMotorOutputs(leftMotorSpeed, rightMotorSpeed);
    }

    /**
     * Arcade drive implements single stick driving. This function lets you
     * directly provide joystick values from any source.
     *
     * @param moveValue The value to use for fowards/backwards
     * @param rotateValue The value to use for the rotate right/left
     */
    public void arcadeDrive(double moveValue, double rotateValue) {
        this.arcadeDrive(moveValue, rotateValue, true);
    }

    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle) {
        rotation = getRotationPID(rotation);
        mecanumDrive_Cartesian0(x, y, rotation, gyroAngle);
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
    }

    private void mecanumDrive_Orientation(double x, double y, double angle) {
        if (!rotationPIDController.isEnable() || rotationPIDController.getSetpoint() != angle) {
            rotationPIDController.setSetpoint(angle);
            rotationPIDController.enable();
        }

        mecanumDrive_Cartesian0(x, y, -rotationSpeedPID, RobotMap.driveSubsystemSteeringGyro.getAngle());
    }

    /**
     * Drive method for Mecanum wheeled robots.
     *
     * A method for driving with Mecanum wheeled robots. There are 4 wheels on
     * the robot, arranged so that the front and back wheels are toed in 45
     * degrees. When looking at the wheels from the top, the roller axles should
     * form an X across the robot. This is very important.
     *
     * This is designed to be directly driven by joystick axes.
     *
     * @param x The speed that the robot should drive in the X direction.
     * [-1.0..1.0]
     * @param y The speed that the robot should drive in the Y direction. This
     * input is inverted to match the forward == -1.0 that joysticks produce.
     * [-1.0..1.0]
     * @param rotation The rate of rotation for the robot that is completely
     * independent of the translation. [-1.0..1.0]
     * @param gyroAngle The current angle reading from the gyro. Use this to
     * implement field-oriented controls.
     */
    private void mecanumDrive_Cartesian0(double x, double y, double rotation, double gyroAngle) {
        if (!kMecanumCartesian_Reported) {
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_MecanumCartesian);
            kMecanumCartesian_Reported = true;
        }
        double xIn = x;
        double yIn = y;
        // Negate y for the joystick.
        yIn = -yIn;
        // Compenstate for gyro angle.
        double rotated[] = rotateVector(xIn, yIn, gyroAngle);
        xIn = rotated[0];
        yIn = rotated[1];

        double wheelSpeeds[] = new double[kMaxNumberOfMotors];
        wheelSpeeds[MotorType.kFrontLeft_val] = xIn + yIn + rotation;
        wheelSpeeds[MotorType.kFrontRight_val] = -xIn + yIn - rotation;
        wheelSpeeds[MotorType.kRearLeft_val] = -xIn + yIn + rotation;
        wheelSpeeds[MotorType.kRearRight_val] = xIn + yIn - rotation;

        normalize(wheelSpeeds);

        byte syncGroup = (byte) 0x80;

        m_frontLeftMotor.set(wheelSpeeds[MotorType.kFrontLeft_val] * m_invertedMotors[MotorType.kFrontLeft_val] * m_maxOutput, syncGroup);
        m_frontRightMotor.set(wheelSpeeds[MotorType.kFrontRight_val] * m_invertedMotors[MotorType.kFrontRight_val] * m_maxOutput, syncGroup);
        m_rearLeftMotor.set(wheelSpeeds[MotorType.kRearLeft_val] * m_invertedMotors[MotorType.kRearLeft_val] * m_maxOutput, syncGroup);
        m_rearRightMotor.set(wheelSpeeds[MotorType.kRearRight_val] * m_invertedMotors[MotorType.kRearRight_val] * m_maxOutput, syncGroup);

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
     * Drive method for Mecanum wheeled robots.
     *
     * A method for driving with Mecanum wheeled robots. There are 4 wheels on
     * the robot, arranged so that the front and back wheels are toed in 45
     * degrees. When looking at the wheels from the top, the roller axles should
     * form an X across the robot.
     *
     * @param magnitude The speed that the robot should drive in a given
     * direction.
     * @param direction The direction the robot should drive in degrees. The
     * direction and magnitude are independent of the rotation rate.
     * @param rotation The rate of rotation for the robot that is completely
     * independent of the magnitude or direction. [-1.0..1.0]
     */
    public void mecanumDrive_Polar(double magnitude, double direction, double rotation) {
        if (!kMecanumPolar_Reported) {
            UsageReporting.report(UsageReporting.kResourceType_RobotDrive, getNumMotors(), UsageReporting.kRobotDrive_MecanumPolar);
            kMecanumPolar_Reported = true;
        }
        double frontLeftSpeed, rearLeftSpeed, frontRightSpeed, rearRightSpeed;
        // Normalized for full power along the Cartesian axes.
        magnitude = limit(magnitude) * Math.sqrt(2.0);
        // The rollers are at 45 degree angles.
        double dirInRad = (direction + 45.0) * 3.14159 / 180.0;
        double cosD = Math.cos(dirInRad);
        double sinD = Math.sin(dirInRad);

        double wheelSpeeds[] = new double[kMaxNumberOfMotors];
        wheelSpeeds[MotorType.kFrontLeft_val] = (sinD * magnitude + rotation);
        wheelSpeeds[MotorType.kFrontRight_val] = (cosD * magnitude - rotation);
        wheelSpeeds[MotorType.kRearLeft_val] = (cosD * magnitude + rotation);
        wheelSpeeds[MotorType.kRearRight_val] = (sinD * magnitude - rotation);

        normalize(wheelSpeeds);

        byte syncGroup = (byte) 0x80;

        m_frontLeftMotor.set(wheelSpeeds[MotorType.kFrontLeft_val] * m_invertedMotors[MotorType.kFrontLeft_val] * m_maxOutput, syncGroup);
        m_frontRightMotor.set(wheelSpeeds[MotorType.kFrontRight_val] * m_invertedMotors[MotorType.kFrontRight_val] * m_maxOutput, syncGroup);
        m_rearLeftMotor.set(wheelSpeeds[MotorType.kRearLeft_val] * m_invertedMotors[MotorType.kRearLeft_val] * m_maxOutput, syncGroup);
        m_rearRightMotor.set(wheelSpeeds[MotorType.kRearRight_val] * m_invertedMotors[MotorType.kRearRight_val] * m_maxOutput, syncGroup);

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
     * Holonomic Drive method for Mecanum wheeled robots.
     *
     * This is an alias to mecanumDrive_Polar() for backward compatability
     *
     * @param magnitude The speed that the robot should drive in a given
     * direction. [-1.0..1.0]
     * @param direction The direction the robot should drive. The direction and
     * maginitute are independent of the rotation rate.
     * @param rotation The rate of rotation for the robot that is completely
     * independent of the magnitute or direction. [-1.0..1.0]
     */
    void holonomicDrive(float magnitude, float direction, float rotation) {
        mecanumDrive_Polar(magnitude, direction, rotation);
    }

    /**
     * Set the speed of the right and left motors. The motors are set to
     * "leftSpeed" and "rightSpeed". This method does not account for motor
     * flipping like {@link RobotDrive} because this causes a hard to find bug
     * and motor flipping is supposed to be taken care of by the inverted motor
     * settings.
     *
     * @param leftOutput The speed to send to the left side of the robot.
     * @param rightOutput The speed to send to the right side of the robot.
     */
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
     * Limit motor values to the -1.0 to +1.0 range.
     *
     * @param num the number to limit
     * @return the limited number
     */
    protected static double limit(double num) {
        if (num > 1.0) {
            return 1.0;
        }
        if (num < -1.0) {
            return -1.0;
        }
        return num;
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than
     * 1.0.
     *
     * @param wheelSpeeds the array of wheel speeds to normalize
     */
    protected static void normalize(double wheelSpeeds[]) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        int i;
        for (i = 1; i < kMaxNumberOfMotors; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (i = 0; i < kMaxNumberOfMotors; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

    /**
     * Rotate a vector in Cartesian space.
     *
     * @param x the x component of the vector
     * @param y the x component of the vector
     * @param angle the angle to rotate the vector
     * @return a 2 element array containing the rotated vector
     *
     */
    protected static double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle * (3.14159 / 180.0));
        double sinA = Math.sin(angle * (3.14159 / 180.0));
        double out[] = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        return out;
    }

    /**
     * Invert a motor direction. This is used when a motor should run in the
     * opposite direction as the drive code would normally run it. Motors that
     * are direct drive would be inverted, the drive code assumes that the
     * motors are geared with one reversal.
     *
     * @param motor The motor index to invert.
     * @param isInverted True if the motor should be inverted when operated.
     */
    public void setInvertedMotor(MotorType motor, boolean isInverted) {
        m_invertedMotors[motor.value] = isInverted ? -1 : 1;
    }

    /**
     * Set the turning sensitivity.
     *
     * This only impacts the drive() entry-point.
     *
     * @param sensitivity Effectively sets the turning sensitivity (or turn
     * radius for a given value)
     */
    public void setSensitivity(double sensitivity) {
        m_sensitivity = sensitivity;
    }

    /**
     * Configure the scaling factor for using RobotDrive with motor controllers
     * in a mode other than PercentVbus.
     *
     * @param maxOutput Multiplied with the output percentage computed by the
     * drive functions.
     */
    public void setMaxOutput(double maxOutput) {
        m_maxOutput = maxOutput;
    }

    /**
     * Free the speed controllers if they were allocated locally
     */
    public void free() {
        if (m_allocatedSpeedControllers) {
            if (m_frontLeftMotor != null) {
                ((PWM) m_frontLeftMotor).free();
            }
            if (m_frontRightMotor != null) {
                ((PWM) m_frontRightMotor).free();
            }
            if (m_rearLeftMotor != null) {
                ((PWM) m_rearLeftMotor).free();
            }
            if (m_rearRightMotor != null) {
                ((PWM) m_rearRightMotor).free();
            }
        }
    }

    public void setExpiration(double timeout) {
        m_safetyHelper.setExpiration(timeout);
    }

    public double getExpiration() {
        return m_safetyHelper.getExpiration();
    }

    public boolean isAlive() {
        return m_safetyHelper.isAlive();
    }

    public boolean isSafetyEnabled() {
        return m_safetyHelper.isSafetyEnabled();
    }

    public void setSafetyEnabled(boolean enabled) {
        m_safetyHelper.setSafetyEnabled(enabled);
    }

    public String getDescription() {
        return "Robot Drive";
    }

    public void stopMotor() {
        if (m_frontLeftMotor != null) {
            m_frontLeftMotor.set(0.0);
        }
        if (m_frontRightMotor != null) {
            m_frontRightMotor.set(0.0);
        }
        if (m_rearLeftMotor != null) {
            m_rearLeftMotor.set(0.0);
        }
        if (m_rearRightMotor != null) {
            m_rearRightMotor.set(0.0);
        }
    }

    private void setupMotorSafety() {
        m_safetyHelper = new MotorSafetyHelper(this);
        m_safetyHelper.setExpiration(kDefaultExpirationTime);
        m_safetyHelper.setSafetyEnabled(true);
    }

    protected int getNumMotors() {
        int motors = 0;
        if (m_frontLeftMotor != null) {
            motors++;
        }
        if (m_frontRightMotor != null) {
            motors++;
        }
        if (m_rearLeftMotor != null) {
            motors++;
        }
        if (m_rearRightMotor != null) {
            motors++;
        }
        return motors;
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

    /**
     *
     * @param motorSpeeds
     */
    public void setMotorOutputs(double[] motorSpeeds) {
        if (m_frontLeftMotor == null || m_rearLeftMotor == null || m_frontRightMotor == null || m_rearRightMotor == null) {
            throw new NullPointerException("Null motor provided");
        }

        byte syncGroup = (byte) 0x80;

        m_frontLeftMotor.set(limit(motorSpeeds[MotorType.kFrontLeft_val]) * m_invertedMotors[MotorType.kFrontLeft_val] * m_maxOutput, syncGroup);
        m_rearLeftMotor.set(limit(motorSpeeds[MotorType.kRearLeft_val]) * m_invertedMotors[MotorType.kRearLeft_val] * m_maxOutput, syncGroup);

        m_frontRightMotor.set(limit(motorSpeeds[MotorType.kFrontRight_val]) * m_invertedMotors[MotorType.kFrontRight_val] * m_maxOutput, syncGroup);
        m_rearRightMotor.set(limit(motorSpeeds[MotorType.kRearRight_val]) * m_invertedMotors[MotorType.kRearRight_val] * m_maxOutput, syncGroup);

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
}
