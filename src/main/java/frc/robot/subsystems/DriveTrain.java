/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.wpilibj.XboxController.Button;


/**
 * Add your docs here.
 */
public class DriveTrain extends SubsystemBase {
  // Sets left motors to their corresponding configurations
  private CANSparkMax motorLeft0 = new CANSparkMax(Constants.MOTOR_LEFT_ZERO, MotorType.kBrushless);
  private CANSparkMax motorLeft1 = new CANSparkMax(Constants.MOTOR_LEFT_ONE, MotorType.kBrushless);
  private CANSparkMax motorLeft2 = new CANSparkMax(Constants.MOTOR_LEFT_TWO, MotorType.kBrushless);
  
  // Sets right motors to their corresponding configurations
  private CANSparkMax motorRight0 = new CANSparkMax(Constants.MOTOR_RIGHT_ZERO, MotorType.kBrushless);
  private CANSparkMax motorRight1 = new CANSparkMax(Constants.MOTOR_RIGHT_ONE, MotorType.kBrushless);
  private CANSparkMax motorRight2 = new CANSparkMax(Constants.MOTOR_RIGHT_TWO, MotorType.kBrushless);

  // These fields hold grouped motors
  private SpeedControllerGroup motorLeft = new SpeedControllerGroup(motorLeft0, motorLeft1, motorLeft2);
  private SpeedControllerGroup motorRight = new SpeedControllerGroup(motorRight0, motorRight1, motorRight2);

  // Differential drive method
  DifferentialDrive diffDrive = new DifferentialDrive(motorLeft, motorRight);

  public DriveTrain() { 
    // Inverts left motors
    motorLeft.setInverted(Constants.LEFT_INVERSION);
    motorRight.setInverted(Constants.RIGHT_INVERSION);

    // Sets deadband for better joystick performance
    diffDrive.setDeadband(Constants.DEADBAND);

    // Sets all motors to zero
    motorLeft.set(0);
    motorRight.set(0);
  }

  // Boilerplates speed to avoid errors
  public double boilerSpeed(double speed) {
    // Sets speed within -1 and 1
    if(Math.abs(speed) > 1) {
      speed = speed / Math.abs(speed);
    }
    return speed;
  }
  
  // Sets up left motors
  public void leftMotor(double speed) {
    // Keeps speed within -1 and 1
    speed = boilerSpeed(speed);

    // Sets all left motors to same speed
    motorLeft.set(speed * Constants.SPEED_MULTIPLIER);
  }

  // Sets up right motors
  public void rightMotor(double speed) {
    // Keeps speed within -1 and 1
    speed = boilerSpeed(speed);

    // Sets all right motors to same speed
    motorRight.set(speed * Constants.SPEED_MULTIPLIER);
  }

  // Moves motors based on speed
  public void driveSpeed(double leftSpeed, double rightSpeed) {
    // Sets power of motors
    leftMotor(leftSpeed);
    rightMotor(rightSpeed);
  }

  // Stops all motors
  public void stopSpeed() {
    // Sets all motor power to zero
    diffDrive.stopMotor();
  }

  // Arcade Drive
  public void drive(double linVelocity, double rotVelocity) {
    /* Uses arcade drive and squares input by stating 
       true in other to obtain better movement */
   diffDrive.arcadeDrive(linVelocity, rotVelocity, true);
     //diffDrive.curvatureDrive(linVelocity, rotVelocity, RobotContainer.driveController.getAButton());
  }

/*
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
      this.setDefaultCommand(new DriveCommand());
  } 
*/

}
