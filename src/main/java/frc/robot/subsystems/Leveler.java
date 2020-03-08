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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SparkConstants;
import frc.robot.utils.LightStripBlinkin;

public class Leveler extends SubsystemBase {
  /**
   * For Our Robot, We Use Wheels, One Thats Stationary for Stability
   * and Another to Move Around While We Are On The Generator Switch
   */

  // Holds the Micro NavX for Leveling
  private AHRS navX;

  // Holds the Motor that Moves on Beam
  private final CANSparkMax motor;

  // Holds Encoders to Measure Velocity
  private final CANEncoder encoder;


  /* LEVELER CONSTRUCTOR */
  public Leveler() {
    // Micro NavX Communication with I2C
    try {
      navX = new AHRS(I2C.Port.kMXP);
    } catch (final RuntimeException ex) {
      DriverStation.reportError("Error instantiating NavX MXP: " + ex.getMessage(), true);
    }

  /* LEVELER METHODS */
    // Sets Up Motor
    motor = new CANSparkMax(SparkConstants.MOTOR_LEVELER, MotorType.kBrushless);

    // Encoders Started
    encoder = motor.getEncoder();
  }

  public void move(double velocity) {
    // Moves Motor
    if(Math.abs(velocity) > 0.2) {
      motor.set(velocity);
    }
    else {
      motor.set(0);
    }
  }

  public void stopMovement() {
    // Stops Motor
    motor.stopMotor();
  }

  public double getAngle() {
    // Returns Roll of the Gyro Sensor
    return navX.getRoll();
  }

  public double rotationVelocity() {
    // Returns Rotation per Second of Motors
    return encoder.getVelocity();
  }

  public void printData() {
    // The Angle of the Leveler
    SmartDashboard.putNumber("Level Angle", getAngle());

    // Is it Leveled?
    SmartDashboard.putBoolean("Leveled", getAngle() < 8 && getAngle() > -8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    if(Math.abs(getAngle()) < 8) {
        //LightStripBlinkin.twinklesOcean();
    }
  }
}
