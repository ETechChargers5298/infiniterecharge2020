/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SolenoidConstants;
import frc.robot.Constants.SparkConstants;

/**
   * Creates a new DriveTrainAdvanced.
   */
public class DriveTrain extends SubsystemBase {
  

  /*DRIVETRAIN ADVANCED FIELDS */

  // Holds the Left Motors
  private CANSparkMax motorLeftA;
  private CANSparkMax motorLeftB;

  // Holds the Right Motors
  private CANSparkMax motorRightA;
  private CANSparkMax motorRightB;

  // Groups Motors Together
  private SpeedControllerGroup motorLeft;
  private SpeedControllerGroup motorRight;

  // Holds the Encoders
  private CANEncoder encoderLeft;
  private CANEncoder encoderRight;

  // PID Controllers Use Encoders to Fix Velocity
  private PIDController speedLeftController;
  private PIDController speedRightController;
  private PIDController torqueLeftController;
  private PIDController torqueRightController;

  // Holds Navigational Board (IMU)
  private AHRS navX;

  // Uses Motors for Differential Drive
  private DifferentialDrive diffDrive;

  // Uses Motors for Differential Kinematics
  private DifferentialDriveKinematics diffKinematics;

  // Uses Gyro and Encoders to Create Pose2D of Robot
  private DifferentialDriveOdometry diffOdometry;

  // Holds the Current Position of the Robot on Field
  Pose2d location;

  // Implements FeedForward to Account for Friction
  private SimpleMotorFeedforward speedFeedforward;
  private SimpleMotorFeedforward torqueFeedforward;

  // Slew Rate Limiters for Joystick
  private SlewRateLimiter speedLimiter;
  private SlewRateLimiter rotLimiter;

  // Holds Gear Shifting Solenoids
  private DoubleSolenoid gearShift;
  
  // Holds Data for Current DriveMode
  private boolean isHighTorque;
  
  /* DRIVETRAIN ADVANCED CONSTRUCTOR */

  public DriveTrain() {
    // Constructs Left Motors
    motorLeftA = new CANSparkMax(SparkConstants.MOTOR_LEFT_A, MotorType.kBrushless);
    motorLeftB = new CANSparkMax(SparkConstants.MOTOR_LEFT_B, MotorType.kBrushless);

    // Constructs Right Motors
    motorRightA = new CANSparkMax(SparkConstants.MOTOR_RIGHT_A, MotorType.kBrushless);
    motorRightB = new CANSparkMax(SparkConstants.MOTOR_RIGHT_B, MotorType.kBrushless);

    // Groups Motors Together for Differential Drive
    motorLeft = new SpeedControllerGroup(motorLeftA, motorLeftB);
    motorRight = new SpeedControllerGroup(motorRightA, motorRightB);

    // Inverts Motors
    motorLeft.setInverted(DriveConstants.LEFT_INVERSION);
    motorRight.setInverted(DriveConstants.RIGHT_INVERSION);

    // Obtains Encoders from Speed Controllers
    encoderLeft = motorLeftA.getAlternateEncoder(AlternateEncoderType.kQuadrature, DriveConstants.DRIVE_ENCODER_RESOLUTION);
    encoderRight = motorRightA.getAlternateEncoder(AlternateEncoderType.kQuadrature, DriveConstants.DRIVE_ENCODER_RESOLUTION);

    // Resets Encoders
    resetLeftPosition();
    resetRightPosition();

    // Inverts Encoders if Needed
    encoderLeft.setInverted(DriveConstants.LEFT_ENCODER_INVERSION);
    encoderRight.setInverted(DriveConstants.RIGHT_ENCODER_INVERSION);

    // Initializes PID Controllers for Each Wheel
    speedLeftController = new PIDController(DriveConstants.LEFT_SPEED_DRIVE_P, DriveConstants.LEFT_SPEED_DRIVE_I, DriveConstants.LEFT_SPEED_DRIVE_D);
    speedRightController = new PIDController(DriveConstants.RIGHT_SPEED_DRIVE_P, DriveConstants.RIGHT_SPEED_DRIVE_I, DriveConstants.RIGHT_SPEED_DRIVE_D);
    torqueLeftController = new PIDController(DriveConstants.LEFT_TORQUE_DRIVE_P, DriveConstants.LEFT_TORQUE_DRIVE_I, DriveConstants.LEFT_TORQUE_DRIVE_D);
    torqueRightController = new PIDController(DriveConstants.RIGHT_TORQUE_DRIVE_P, DriveConstants.RIGHT_TORQUE_DRIVE_I, DriveConstants.RIGHT_TORQUE_DRIVE_D);

    // Connects to NavX
    try {
      navX = new AHRS(SPI.Port.kMXP);
    } catch(RuntimeException ex) {
      // Error Printed if Rio Cannot Connect
      DriverStation.reportError("Error instantiating NavX MXP: " + ex.getMessage(), true);
    }

    // Reset Yaw so Starting Heading is Zero Degrees
    resetGyro();

    // Constructs a Differential Drive to Move Robot
    diffDrive = new DifferentialDrive(motorLeft, motorRight);

    // Constructs Kinematics to Switch Between Wheel Speed and Chassis Speed
    diffKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveConstants.DRIVE_TRACK_WIDTH));

    // Constructs Odometry for Tracking Robot on Field
    diffOdometry = new DifferentialDriveOdometry(getHeading());

    // Location of the Robot on Field
    location = new Pose2d();

    // Constructs feedForward to Account for Friction
    speedFeedforward = new SimpleMotorFeedforward(DriveConstants.DRIVE_SPEED_STATIC_GAIN, DriveConstants.DRIVE_SPEED_VELOCITY_GAIN);
    torqueFeedforward = new SimpleMotorFeedforward(DriveConstants.DRIVE_TORQUE_STATIC_GAIN, DriveConstants.DRIVE_TORQUE_VELOCITY_GAIN);

    // Creates Slew Rate Limitations Which Limits Rate of Change
    speedLimiter = new SlewRateLimiter(JoystickConstants.SPEED_LIMIT);
    rotLimiter = new SlewRateLimiter(JoystickConstants.ROT_LIMIT);

    // Sets Deadband for Driving
    diffDrive.setDeadband(JoystickConstants.DEADBAND);

    // Constructs a DoubleSolenoid to Shift Gears in GearBox
    gearShift = new DoubleSolenoid(SolenoidConstants.SHIFTER_PORT_A, SolenoidConstants.SHIFTER_PORT_B);
  
    // Begins with High Torque at Start of Match
    highTorque();
  }

  // Drive Robot Using Power Range from -1 to 1
  public void powerDrive(double leftSpeed, double rightSpeed) {
    // Clips Speed to Stay Within Range
    leftSpeed = MathUtil.clamp(leftSpeed, -1 * DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED);
    rightSpeed = MathUtil.clamp(rightSpeed, -1 * DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED);

    // Moves Robot
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  // Drive Robot With FeedForward Implementation
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Sets Up Variables to Hold Feedforward Calculations
    double leftFeedforward;
    double rightFeedforward;

    // Sets Up Variables to Get Current Speed and Compares to Desired Speed
    double leftOutput;
    double rightOutput;

    // Calculate Feedforward and Output Depending on Gear Mode
    if(isHighTorque) {
      // Feedforward Calculations
      leftFeedforward = torqueFeedforward.calculate(speeds.leftMetersPerSecond);
      rightFeedforward = torqueFeedforward.calculate(speeds.rightMetersPerSecond);

      // Output Calculations
      leftOutput = torqueLeftController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
      rightOutput = torqueRightController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);
    }
    else {
      // Feedforward Calculations
      leftFeedforward = speedFeedforward.calculate(speeds.leftMetersPerSecond);
      rightFeedforward = speedFeedforward.calculate(speeds.rightMetersPerSecond);

      // Output Calculations
      leftOutput = speedLeftController.calculate(getLeftVelocity(), speeds.leftMetersPerSecond);
      rightOutput = speedRightController.calculate(getRightVelocity(), speeds.rightMetersPerSecond);
    }

    // Applys Calculated Voltage to Motors
    motorLeft.setVoltage(leftOutput + leftFeedforward);
    motorRight.setVoltage(rightOutput + rightFeedforward);
  }

  // Drives Using Linear Velocity and Angular Velocity
  public void velocityDrive(double linear, double rotational) {
    // Converts Velocity to Wheel Speed
    var wheelSpeeds = diffKinematics.toWheelSpeeds(new ChassisSpeeds(linear, 0, rotational));

    // Sets Converted Wheel Speeds
    setSpeeds(wheelSpeeds);
  }

  // Drives Using Arcade Drive from Differencial Drive Class
  public void arcadeDrive(double lin, double rot) {
    diffDrive.arcadeDrive(lin, rot);
  }

  // Moves Motor By Stating the Voltage of the Motors
  public void voltDrive(double leftVolts, double rightVolts) {
    motorLeft.setVoltage(leftVolts);
    motorRight.setVoltage(rightVolts);
  }

  // Drives Using Custom Drive Accounting for Friction
  public void advancedDrive(double linVelocity, double rotVelocity) {
    // Applies Slew Rate Limiters to Inputs
    linVelocity = -speedLimiter.calculate(linVelocity) * DriveConstants.MAX_VELOCITY;
    rotVelocity = -rotLimiter.calculate(rotVelocity) * DriveConstants.MAX_TURN_VELOCITY;

    // Uses Updated Velocities to Drive
    velocityDrive(linVelocity, rotVelocity);
  }

  // Stops DriveTrain Motors
  public void stopDrive() {
    diffDrive.stopMotor();
  }

  // Get the Left Wheel Position of the Robot in Meters
  public double getLeftPosition() {
    return encoderLeft.getPosition() * Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_CIRCUMFERENCE);
  }

  // Get the Right Wheel Position of the Robot in Meters
  public double getRightPosition() {
    return encoderRight.getPosition() * Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_CIRCUMFERENCE);
  }

  // Resets the Left Wheel Position
  public void resetLeftPosition() {
    encoderLeft.setPosition(0);
  }

  // Resets the Right Wheel Position
  public void resetRightPosition() {
    encoderRight.setPosition(0);
  }

  // Get the Left Wheel Velocity of the Robot in Meters per Second
  public double getLeftVelocity() {
    return encoderLeft.getVelocity() * Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_CIRCUMFERENCE) / 60;
  }

  // Get the Right Wheel Velocity of the Robot in Meters per Second
  public double getRightVelocity() {
    return encoderRight.getVelocity() * Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_CIRCUMFERENCE) / 60;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  /* DRIVETRAIN ADVANCED METHODS */

  // Resets Gyro Angle
  public void resetGyro() {
    navX.reset();
  }

  // Returns Angle as Rotation2d
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  // Returns Angle as Double
  public double getAngle() {
    return Math.IEEEremainder(navX.getAngle(), 360);
  }

  // Returns the PID Controller for Left Wheels (High Speed)
  public PIDController getLeftSpeedPID() {
    return speedLeftController;
  }

  // Returns the PID Controller for Right Wheels (High Speed)
  public PIDController getRightSpeedPID() {
    return speedRightController;
  }

  // Returns the PID Controller for Left Wheels (High Torque)
  public PIDController getLeftTorquePID() {
    return torqueLeftController;
  }

  // Returns the PID Controller for Right Wheel (High Torque)
  public PIDController getRightTorquePID() {
    return torqueRightController;
  }

  // Gets the Feedforward for High Torque
  public SimpleMotorFeedforward getTorqueFeedforward() {
    return torqueFeedforward;
  }

  // Gets the Feedforward for High Speed
  public SimpleMotorFeedforward getSpeedFeedforward() {
    return speedFeedforward;
  }

  // Gets Kinematics of DriveTrain
  public DifferentialDriveKinematics getKinematics() {
    return diffKinematics;
  }

  // Gets Position of Robot
  public Pose2d getPose() {
    return location;
  }

  // Gear Shifts to High Torque
  public void highTorque() {
    // Sends Air to High Torque Pipes
    gearShift.set(Value.kForward);
    
    // Updates DriveMode
    isHighTorque = true;
  }

  // Gear Shifts to High Speed
  public void highSpeed() {
    // Sends Air to High Speed Pipes
    gearShift.set(Value.kReverse);

    // Updates DriveMode
    isHighTorque = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Updates Odometry Constantly
    location = diffOdometry.update(getHeading(), getLeftPosition(), getRightPosition());
  }
}
