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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
import frc.robot.utils.LimeLight;

public class DriveTrain extends SubsystemBase {
  
  /* DRIVETRAIN FIELDS */

  // Hold the Left Motors
  private CANSparkMax motorLeftA;
  private CANSparkMax motorLeftB;

  // Hold the Right Motors
  private CANSparkMax motorRightA;
  private CANSparkMax motorRightB;

  // Holds Grouped Motors
  private SpeedControllerGroup motorLeft;
  private SpeedControllerGroup motorRight;

  // PID Controllers for Drive
  private PIDController leftController;
  private PIDController rightController;

  // Implements FeedForward Constraint DriveTrain
  private SimpleMotorFeedforward feedForward;

  // Slew Rate Limiters for Joystick
  private SlewRateLimiter speedLimiter;
  private SlewRateLimiter rotLimiter;

  // Uses Motors for Differential Kinematics
  private DifferentialDrive diffDrive;

  // Uses Motors for Differential Drive 
  private DifferentialDriveKinematics diffKinematics;

  // Creates a Odometry 
  private DifferentialDriveOdometry diffOdometry;

  // Holds Two Encoders
  private CANEncoder encoderLeft;
  private CANEncoder encoderRight;

  // Holds GearShifting Solenoids
  private DoubleSolenoid gearShift;

  // Holds Data For Current DriveMode
  private boolean isHighTorque;

  // Holds Navigation Board (IMU)
  private AHRS navX;

  private LimeLight lime = new LimeLight();

  public DriveTrain() {
    // Constructs Left Motors
    motorLeftA = new CANSparkMax(SparkConstants.MOTOR_LEFT_A, MotorType.kBrushless);
    motorLeftB = new CANSparkMax(SparkConstants.MOTOR_LEFT_B, MotorType.kBrushless);
    
    // Constructs Right Motors
    motorRightA = new CANSparkMax(SparkConstants.MOTOR_RIGHT_A, MotorType.kBrushless);
    motorRightB = new CANSparkMax(SparkConstants.MOTOR_RIGHT_B, MotorType.kBrushless);

    motorLeftA.setInverted(DriveConstants.LEFT_INVERSION); // Test Difference On Impact
    motorLeftB.setInverted(DriveConstants.LEFT_INVERSION);
    motorRightA.setInverted(DriveConstants.RIGHT_INVERSION);
    motorRightB.setInverted(DriveConstants.RIGHT_INVERSION);

    // Groups Motors Together for Differential Drive
    motorLeft = new SpeedControllerGroup(motorLeftA, motorLeftB);
    motorRight = new SpeedControllerGroup(motorRightA, motorRightB);

    // Inverts Motors
    //motorLeft.setInverted(DriveConstants.LEFT_INVERSION);
    //motorRight.setInverted(DriveConstants.RIGHT_INVERSION);

    // PID Controllers are Initialized
    leftController = new PIDController(DriveConstants.LEFT_SPEED_DRIVE_P, DriveConstants.LEFT_SPEED_DRIVE_I, DriveConstants.LEFT_SPEED_DRIVE_D);
    rightController = new PIDController(DriveConstants.RIGHT_SPEED_DRIVE_P, DriveConstants.RIGHT_SPEED_DRIVE_I, DriveConstants.RIGHT_SPEED_DRIVE_D);

    // Implements Feedforward to Account for Physics
    feedForward = new SimpleMotorFeedforward(DriveConstants.DRIVE_TORQUE_STATIC_GAIN, DriveConstants.DRIVE_TORQUE_VELOCITY_GAIN);

    // Slew Rate Limitation
    speedLimiter = new SlewRateLimiter(JoystickConstants.SPEED_LIMIT);
    rotLimiter = new SlewRateLimiter(JoystickConstants.ROT_LIMIT);

    // Creates a Differential Kinematics for Chassis
    diffKinematics = new DifferentialDriveKinematics(DriveConstants.DRIVE_TRACK_WIDTH);

    // Creates a Differential Drive Object Using Grouped Motors
    diffDrive = new DifferentialDrive(motorLeft, motorRight);

    // Creates Encoder objects
    //encoderLeft = motorLeft0.getAlternateEncoder(AlternateEncoderType.kQuadrature, DriveConstants.DRIVE_ENCODER_RESOLUTION);
    //encoderRight = motorRight0.getAlternateEncoder(AlternateEncoderType.kQuadrature, DriveConstants.DRIVE_ENCODER_RESOLUTION);
    encoderLeft = motorLeftA.getEncoder();
    encoderRight = motorRightA.getEncoder();

    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);

    // Sets Factors for Position to Measure in Meters
    encoderLeft.setPositionConversionFactor(Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_CIRCUMFERENCE));
    encoderRight.setPositionConversionFactor(Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_CIRCUMFERENCE));

    // Sets Factors for Velocity to Measure in Meters per Second
    encoderLeft.setVelocityConversionFactor(Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_CIRCUMFERENCE));
    encoderRight.setVelocityConversionFactor(Units.inchesToMeters(DriveConstants.DRIVE_WHEEL_CIRCUMFERENCE));

    // Creates DoubleSolonoid to Shift Gears in GearBox
    gearShift = new DoubleSolenoid(SolenoidConstants.SHIFTER_PORT_A, SolenoidConstants.SHIFTER_PORT_B);

    // Begins with High Torque Everytime
    // Updates DriveMode
    isHighTorque = true;

    // Connects to NavX
    try {
      navX = new AHRS(SPI.Port.kMXP);
    } catch(RuntimeException ex) {
      // Error Printed to Driver Station
      DriverStation.reportError("Error instantiating NavX MXP: " + ex.getMessage(), true);
    }

    // Resets Yaw to Start Heading of Robot During GamePlay
    navX.reset();

    // Sets Up Odometry
    diffOdometry = new DifferentialDriveOdometry(getAngle());

    //diffDrive.setDeadband(JoystickConstants.DEADBAND);
  }


  /* DRIVETRAIN METHODS */

  // For Arcade Drive Joysticks
  public void arcadeDrive(double linVelocity, double rotVelocity) {
    // Changes Speed to Match Sensitivity 
    //linVelocity = Math.copySign(Math.pow(linVelocity, JoystickConstants.JOYSTICK_SENSITIVITY), linVelocity);
    //rotVelocity = Math.copySign(Math.pow(rotVelocity, JoystickConstants.JOYSTICK_SENSITIVITY), rotVelocity);
    // linVelocity = Math.copySign(linVelocity, linVelocity);
    // rotVelocity = Math.copySign(rotVelocty, rotVelocity);

    // Drives Robot
    diffDrive.arcadeDrive(linVelocity, rotVelocity);
  }

  // Test Arcade Drive
  public void arcadeDrive2(double linVelocity, double rotVelocity) {
    // Updates Speeds with Limiters
    var linearVelocity = -speedLimiter.calculate(linVelocity * DriveConstants.MAX_VELOCITY);
    var rotationVelocity = -rotLimiter.calculate(rotVelocity * DriveConstants.MAX_TURN_VELOCITY);

    // Uses Velocity to Drive
    drive(linearVelocity, rotationVelocity);
  }

  // Moves Motors Based on Speed Given
  public void driveSpeed(double leftSpeed, double rightSpeed) {
    // Clamps Values to Acceptable Range
    leftSpeed = MathUtil.clamp(leftSpeed, -1 * DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED);
    rightSpeed = MathUtil.clamp(rightSpeed, -1 * DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED);

    // Sets Speed Which is Impacted by Speed Multiplier
    motorLeft.set(leftSpeed);
    motorRight.set(rightSpeed);
  }

  // Sets Speed of Motors Using a PID Controller
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Feed Forward Calculated for Each Wheel
    double leftFeedForward = feedForward.calculate(speeds.leftMetersPerSecond);
    double rightFeedForward = feedForward.calculate(speeds.rightMetersPerSecond);
    
    // Calculates Output Needed using PID Controller
    double leftOutput = leftController.calculate(encoderLeft.getVelocity(), speeds.leftMetersPerSecond);
    double rightOutput = rightController.calculate(encoderRight.getVelocity(), speeds.rightMetersPerSecond);

    // Sets Voltage to Run Code
    motorLeft.setVoltage(leftOutput + leftFeedForward);
    motorRight.setVoltage(rightOutput + rightFeedForward);
  }

  // Drive Using PID and Real World Velocity
  public void drive(double linVelocity, double rotVelocity) {
    // Converts Real World Velocity to Wheel Power
    var wheelSpeeds = diffKinematics.toWheelSpeeds(new ChassisSpeeds(linVelocity, 0, rotVelocity));

    // Sets Motor Speeds to Move
    setSpeeds(wheelSpeeds);
  }

  // Stops All Motors
  public void stopDrive() {
    diffDrive.stopMotor();
  }

  // Get Right Encoder Values -JG
  public double getEncoderRightValue() {
    return encoderRight.getPosition();
  }

  // Get Left Encoder Values -JG
  public double getEncoderLeftValue() {
    return encoderLeft.getPosition();
  }


  // Gear Shifts to High Torque
  public void highTorque() {
    // Sends Air to High Torque Pipes
    gearShift.set(Value.kForward);
  }

  // Gear Shifts to High Speed
  public void highSpeed() {
    // Sends Air to High Speed Pipes
    gearShift.set(Value.kReverse);
  }

  // Toggles Between Gear Shifts
  public void toggleDriveMode() {
    // Alternates Between True and False
    isHighTorque = !isHighTorque;

    // Calls Method to Match isHighTorque
    if(isHighTorque) {
      highTorque();
    }
    else {
      highSpeed();
    }
  }

  // Returns the Angle The Robot is Facing
  public double getHeading() {
    return Math.IEEEremainder(navX.getAngle(), 360);
  }

  // Returns 2D Angle
  public Rotation2d getAngle() {
    // Returns Angle as a Rotational 2D
    return Rotation2d.fromDegrees(-navX.getAngle());
  }

  // Returns the Degrees per Second of Rotation
  public double getTurnRate() {
    return navX.getRate();
  }

  // Updates Odometry
  public void updateOdometry() {
    diffOdometry.update(getAngle(), encoderLeft.getPosition(), encoderRight.getPosition());
  }

  // Prints Out Data Relating to DriveTrain
  public void printData() {
    // The Velocities of Our Wheels
    SmartDashboard.putData("DriveSpeed", diffDrive);

    // The Current DriveMode We are At
    if(isHighTorque) {
      SmartDashboard.putString("DriveMode", "Torque");
    }
    else {
      SmartDashboard.putString("DriveMode", "Speed");
    }
    
    // The Gyro from NavX
    SmartDashboard.putData("Gyro", navX);

    // Gives Velocity From Encoders
    SmartDashboard.putNumber("Left Encoder Velocity", encoderLeft.getVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", encoderRight.getVelocity());

    // Gives Position From Encoders
    SmartDashboard.putNumber("Left Encoder Position", -1 * encoderLeft.getPosition() / 33 * 2 * Math.PI * 3.5);
    SmartDashboard.putNumber("Right Encoder Position", encoderRight.getPosition() / 33 * 2 * Math.PI * 3.5);
  }

  // Resets Sensors
  public void reset() {
    // Resets Encoders
    encoderLeft.setPosition(0);
    encoderRight.setPosition(0);

    // Resets NavX
    navX.zeroYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    printData();
    lime.updateLimeLight();
    lime.printData();
  }
}