/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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

  // Uses Motors as Differential Drive
  private DifferentialDrive diffDrive;

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

    // Creates a Differential Drive Object Using Grouped Motors
    diffDrive = new DifferentialDrive(motorLeft, motorRight);

    // Sets Safety to the Motors
    diffDrive.setSafetyEnabled(true);

    // Sets Deadband for Better Joystick Performance
    diffDrive.setDeadband(JoystickConstants.DEADBAND);

    // Stops All Motors for Safety
    diffDrive.stopMotor();

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
    //linVelocity = Math.copySign(Math.pow(linVelocity, JoystickConstants.JOYSTICK_SENSITIVITY), linVelocity);
    //rotVelocity = Math.copySign(Math.pow(rotVelocity, JoystickConstants.JOYSTICK_SENSITIVITY), rotVelocity);
    
    // Drives Robot
    diffDrive.arcadeDrive(linVelocity, rotVelocity, true);
    SmartDashboard.putData("Differential Drive", diffDrive);
  }

  // Moves Motors Based on Speed Given
  public void driveSpeed(double leftSpeed, double rightSpeed) {
    // Clamps Values to Acceptable Range
    leftSpeed = MathUtil.clamp(leftSpeed, -0.5, 0.5);
    rightSpeed = MathUtil.clamp(rightSpeed, -0.5, 0.5);

    SmartDashboard.putNumber("LeftWheel", leftSpeed);
    SmartDashboard.putNumber("RightWheel", rightSpeed);

    // Sets Speed Which is Impacted by Speed Multiplier
    motorLeft.set(leftSpeed * DriveConstants.SPEED_MULTIPLIER);
    motorRight.set(rightSpeed * DriveConstants.SPEED_MULTIPLIER);
  }

  // Stops All Motors
  public void stopDrive() {
    diffDrive.stopMotor();
  }

  // Gear Shifts to High Torque
  public void highTorque() {
    // Sends Air to High Torque Pipes
    gearShift.set(Value.kForward);

    // Prints Current DriveMode
    SmartDashboard.putString("DriveMode", "High Torque");
  }

  // Gear Shifts to High Speed
  public void highSpeed() {
    // Sends Air to High Speed Pipes
    gearShift.set(Value.kReverse);

    // Prints Current DriveMode
    SmartDashboard.putString("DriveMode", "High Speed");
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

  // Returns if Robot is at High Torque
  public boolean isHighTorque() {
    return isHighTorque;
  }

  // Returns the Angle The Robot is Facing
  public double getHeading() {
    SmartDashboard.putData("Gyro", navX);
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
