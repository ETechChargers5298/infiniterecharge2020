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
  }

/* @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  } */
}
