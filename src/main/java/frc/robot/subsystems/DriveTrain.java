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
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.robot.Constants.DriveConstants;
import frc.robot.robot.Constants.JoystickConstants;

public class DriveTrain extends SubsystemBase {
  /**
   * Contains the Differential Drive capabilties our robot has
   */

  // Hold the Left Motors
  private CANSparkMax motorLeft0;
  private CANSparkMax motorLeft1;

  // Hold the Right Motors
  private CANSparkMax motorRight0;
  private CANSparkMax motorRight1;

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

  // Holds Two Encoders
  private CANEncoder encoderLeft;
  private CANEncoder encoderRight;

  // Holds GearShifting Solenoids
  private DoubleSolenoid gearShift;

  // Holds Data For Current DriveMode
  private boolean isHighTorque;

  // Holds Navigation Board (IMU)
  private AHRS navX;

  public DriveTrain() {
    // Constructs Left Motors
    motorLeft0 = new CANSparkMax(DriveConstants.MOTOR_LEFT_ZERO, MotorType.kBrushless);
    motorLeft1 = new CANSparkMax(DriveConstants.MOTOR_LEFT_ONE, MotorType.kBrushless);
    
    // Constructs Right Motors
    motorRight0 = new CANSparkMax(DriveConstants.MOTOR_RIGHT_ZERO, MotorType.kBrushless);
    motorRight1 = new CANSparkMax(DriveConstants.MOTOR_RIGHT_ONE, MotorType.kBrushless);

    // Groups Motors Together for Differential Drive
    motorLeft = new SpeedControllerGroup(motorLeft0, motorLeft1);
    motorRight = new SpeedControllerGroup(motorRight0, motorRight1);

    // Inverts Motors
    motorLeft.setInverted(DriveConstants.LEFT_INVERSION);
    motorRight.setInverted(DriveConstants.RIGHT_INVERSION);

    // PID Controllers are Initialized
    leftController = new PIDController(DriveConstants.LEFT_DRIVE_P, DriveConstants.LEFT_DRIVE_I, DriveConstants.LEFT_DRIVE_D);
    rightController = new PIDController(DriveConstants.RIGHT_DRIVE_P, DriveConstants.RIGHT_DRIVE_I, DriveConstants.RIGHT_DRIVE_D);

    // Implements Feedforward to Account for Physics
    feedForward = new SimpleMotorFeedforward(DriveConstants.DRIVE_STATIC_GAIN, DriveConstants.DRIVE_VELOCITY_GAIN);

    // Slew Rate Limitation
    speedLimiter = new SlewRateLimiter(JoystickConstants.SPEED_LIMIT);
    rotLimiter = new SlewRateLimiter(JoystickConstants.ROT_LIMIT);

    // Creates a Differential Kinematics for Chassis
    diffKinematics = new DifferentialDriveKinematics(DriveConstants.DRIVE_TRACK_WIDTH);

    // Creates a Differential Drive Object Using Grouped Motors
    diffDrive = new DifferentialDrive(motorLeft, motorRight);

    // Creates Encoder objects
    encoderLeft = motorLeft0.getAlternateEncoder(AlternateEncoderType.kQuadrature, DriveConstants.DRIVE_ENCODER_RESOLUTION);
    encoderRight = motorRight0.getAlternateEncoder(AlternateEncoderType.kQuadrature, DriveConstants.DRIVE_ENCODER_RESOLUTION);

    // Creates DoubleSolonoid to Shift Gears in GearBox
    gearShift = new DoubleSolenoid(DriveConstants.SHIFTER_PORT_ONE, DriveConstants.SHIFTER_PORT_TWO);

    // Begins with High Torque Everytime
    highTorque();

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
  }

  // For Arcade Drive Joysticks
  public void arcadeDrive(double linVelocity, double rotVelocity) {
    // Changes Speed to Match Sensitivity 
    linVelocity = Math.copySign(Math.pow(linVelocity, JoystickConstants.JOYSTICK_SENSITIVITY), linVelocity);
    rotVelocity = Math.copySign(Math.pow(rotVelocity, JoystickConstants.JOYSTICK_SENSITIVITY), rotVelocity);
    
    // Drives Robot
    diffDrive.arcadeDrive(linVelocity, rotVelocity);

    // Prints Velocity of Wheels
    SmartDashboard.putData("Differential Drive", diffDrive);
  }

  // Test Arcade Drive
  public void arcadeDrive2(double linVelocity, double rotVelocity) {
    // Updates Speeds with Limiters
    var linearVelocity = -speedLimiter.calculate(linVelocity * DriveConstants.MAX_VELOCITY);
    var rotationVelocity = -rotLimiter.calculate(rotVelocity * DriveConstants.MAX_TURN_SPEED);

    // Uses Velocity to Drive
    drive(linearVelocity, rotationVelocity);
  }

  // Moves Motors Based on Speed Given
  public void driveSpeed(double leftSpeed, double rightSpeed) {
    // Clamps Values to Acceptable Range
    leftSpeed = MathUtil.clamp(leftSpeed, -1 * DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED);
    rightSpeed = MathUtil.clamp(rightSpeed, -1 * DriveConstants.MAX_SPEED, DriveConstants.MAX_SPEED);

    // Sets Speed Which is Impacted by Speed Multiplier
    motorLeft.set(leftSpeed * DriveConstants.SPEED_MULTIPLIER);
    motorRight.set(rightSpeed * DriveConstants.SPEED_MULTIPLIER);
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
    double ri = encoderRight.getPosition();
    SmartDashboard.putNumber("Right Encoder", ri);
    return ri;
  }

  // Get Left Encoder Values -JG
  public double getEncoderLeftValue() {
    double le = encoderLeft.getPosition();
    SmartDashboard.putNumber("Left Encoder", le);
    return le;
  }


  // Gear Shifts to High Torque
  public void highTorque() {
    // Sends Air to High Torque Pipes
    gearShift.set(Value.kForward);

    // Prints Current DriveMode
    SmartDashboard.putString("DriveMode", "Torque");
  }

  // Gear Shifts to High Speed
  public void highSpeed() {
    // Sends Air to High Speed Pipes
    gearShift.set(Value.kReverse);

    // Prints Current DriveMode
    SmartDashboard.putString("DriveMode", "Speed");
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
    // Prints Gyro Heading
    SmartDashboard.putData("Gyro", navX);

    // Prints Graph of Heading over Time
    SmartDashboard.putNumber("GyroTime", navX.getAngle());

    return Math.IEEEremainder(navX.getAngle(), 360);
  }

  // Returns the Degrees per Second of Rotation
  public double getTurnRate() {
    return navX.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}