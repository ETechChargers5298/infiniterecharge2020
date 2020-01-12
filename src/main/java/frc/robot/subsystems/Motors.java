/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Drive;

/**
 * Add your docs here.
 */
public class Motors extends SubsystemBase {
  // These fields will hold motor values
  private CANSparkMax motorLeft0;
  private CANSparkMax motorLeft1;
  private CANSparkMax motorLeft2;
  private CANSparkMax motorRight0;
  private CANSparkMax motorRight1;
  private CANSparkMax motorRight2;

  public Motors() {
    // Sets left motors to their corresponding configurations
    motorLeft0 = new CANSparkMax(Constants.MOTOR_LEFT_ZERO, MotorType.kBrushless);
    motorLeft1 = new CANSparkMax(Constants.MOTOR_LEFT_ONE, MotorType.kBrushless);
    motorLeft2 = new CANSparkMax(Constants.MOTOR_LEFT_TWO, MotorType.kBrushless);

    // Sets left motors to their corresponding configurations
    motorRight0 = new CANSparkMax(Constants.MOTOR_RIGHT_ZERO, MotorType.kBrushless);
    motorRight1 = new CANSparkMax(Constants.MOTOR_RIGHT_ONE, MotorType.kBrushless);
    motorRight2 = new CANSparkMax(Constants.MOTOR_RIGHT_TWO, MotorType.kBrushless);
    
    // Inverts left motors
    motorLeft0.setInverted(Constants.LEFT_INVERSION);
    motorLeft1.setInverted(Constants.LEFT_INVERSION);
    motorLeft2.setInverted(Constants.LEFT_INVERSION);

    // Inverts right motors
    motorRight0.setInverted(Constants.RIGHT_INVERSION);
    motorRight1.setInverted(Constants.RIGHT_INVERSION);
    motorRight2.setInverted(Constants.RIGHT_INVERSION);

    // Sets all motors to zero
  }

  // Boilerplates speed to avoid errors
  public double boilerSpeed(double speed) {
    // Sets speed within -1 and 1
    if(Math.abs(speed) > 1) {
      speed = speed / Math.abs(speed);
    }
    return speed;
  }
  
  // Groups left motors
  public void leftMotor(double speed) {
    // Keeps speed within -1 and 1
    speed = boilerSpeed(speed);

    // Sets all left motors to same speed
    motorLeft0.set(speed);
    motorLeft1.set(speed);
    motorLeft2.set(speed);
  }

  // Groups right motors
  public void rightMotor(double speed) {
    // Keeps speed within -1 and 1
    speed = boilerSpeed(speed);

    // Sets all right motors to same speed
    motorRight0.set(speed);
    motorRight1.set(speed);
    motorRight2.set(speed);
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
