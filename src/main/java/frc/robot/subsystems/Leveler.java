/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Constants.LevelConstants;

public class Leveler extends SubsystemBase {
  /**
   * For Our Robot, We Use Wheels, One Thats Stationary for Stability
   * and Another to Move Around While We Are On The Generator Switch
   */

  // Holds the Micro NavX for Leveling
  private AHRS navX;

  // Holds the Motor that Moves on Beam
  private CANSparkMax motor;

  // Holds Encoders to Measure Velocity
  private CANEncoder encoder;

  public Leveler() {
    // Micro NavX Communication with I2C
    try {
      navX = new AHRS(I2C.Port.kMXP);
    }
    catch(RuntimeException ex) {
      DriverStation.reportError("Error instantiating NavX MXP: " + ex.getMessage(), true);
    }

    // Sets Up Motor
    motor = new CANSparkMax(LevelConstants.LEVEL_MOTOR_ID, MotorType.kBrushless);

    // Encoders Started
    encoder = motor.getEncoder();
  }

  public void move(double speed) {
    // Moves Motor
    motor.set(speed);
  }

  public void stopMovement() {
    // Stops Motor
    motor.stopMotor();
  }

  public double getRoll() {
    // Returns Roll of the Gyro Sensor
    return navX.getRoll();
  }

  public double rotationVelocity() {
    // Returns Rotation per Second of Motors
    return encoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
