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
  private CANSparkMax frontLeft;
  private CANSparkMax frontRight;
  private CANSparkMax backLeft;
  private CANSparkMax backRight;

  public Motors() {
    // Sets motors to their corresponding configurations
    frontLeft = new CANSparkMax(Constants.MOTOR_FRONT_LEFT, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.MOTOR_FRONT_RIGHT, MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.MOTOR_BACK_LEFT, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.MOTOR_BACK_RIGHT, MotorType.kBrushless);
    
    // Inverts motors
    frontLeft.setInverted(Constants.LEFT_INVERSION);
    frontRight.setInverted(Constants.RIGHT_INVERSION);
    backLeft.setInverted(Constants.LEFT_INVERSION);
    backRight.setInverted(Constants.RIGHT_INVERSION);

    // Sets all motors to zero
    frontLeft.set(0);
    frontRight.set(0);
    backLeft.set(0);
    backRight.set(0);
  }

  // Moves motors based on speed
  public void driveSpeed(double leftSpeed, double rightSpeed) {
    // Sets speed within -1 and 1
    if(Math.abs(leftSpeed) > 1) {
      leftSpeed = leftSpeed / Math.abs(leftSpeed);
    }
    if(Math.abs(rightSpeed) > 1) {
      rightSpeed = rightSpeed / Math.abs(rightSpeed);
    }   

    // Sets power of left motors
    frontLeft.set(leftSpeed);
    backLeft.set(leftSpeed);

    // Sets power of right motors
    frontRight.set(rightSpeed);
    backRight.set(rightSpeed);
  }

  // Stops all motors
  public void stopSpeed() {
    // Sets all motor power to zero
    frontLeft.set(0);
    frontRight.set(0);
    backLeft.set(0);
    backRight.set(0);
  }

/*
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
      setDefaultCommand(new Drive());
  } 
*/
}
