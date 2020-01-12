/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drive;

/**
 * Add your docs here.
 */
public class Motors extends SubsystemBase {
  // These fields will hold motors
  private CANSparkMax motorLeft0;
  private CANSparkMax motorLeft1;
  private CANSparkMax motorLeft2;
  private CANSparkMax motorRight0;
  private CANSparkMax motorRight1;
  private CANSparkMax motorRight2;

  // These fields hold grouped motors
  private SpeedControllerGroup motorLeft;
  private SpeedControllerGroup motorRight;

  public Motors() {
    // Sets left motors to their corresponding configurations
    motorLeft0 = new CANSparkMax(Constants.MOTOR_LEFT_ZERO, MotorType.kBrushless);
    motorLeft1 = new CANSparkMax(Constants.MOTOR_LEFT_ONE, MotorType.kBrushless);
    motorLeft2 = new CANSparkMax(Constants.MOTOR_LEFT_TWO, MotorType.kBrushless);

    // Sets left motors to their corresponding configurations
    motorRight0 = new CANSparkMax(Constants.MOTOR_RIGHT_ZERO, MotorType.kBrushless);
    motorRight1 = new CANSparkMax(Constants.MOTOR_RIGHT_ONE, MotorType.kBrushless);
    motorRight2 = new CANSparkMax(Constants.MOTOR_RIGHT_TWO, MotorType.kBrushless);
    
    // Groups motors together
    motorLeft = new SpeedControllerGroup(motorLeft0, motorLeft1, motorLeft2);
    motorRight = new SpeedControllerGroup(motorRight0, motorRight1, motorRight2);

    // Inverts left motors
    motorLeft.setInverted(Constants.LEFT_INVERSION);
    motorRight.setInverted(Constants.RIGHT_INVERSION);

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
    motorLeft.set(speed);
  }

  // Sets up right motors
  public void rightMotor(double speed) {
    // Keeps speed within -1 and 1
    speed = boilerSpeed(speed);

    // Sets all right motors to same speed
    motorRight.set(speed);
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
    leftMotor(0);
    rightMotor(0);
  }

/*
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
      setDefaultCommand(new Drive());
  } 
*/
}
