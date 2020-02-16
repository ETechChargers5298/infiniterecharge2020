/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SparkConstants;

public class Loader extends SubsystemBase {
  /**
   * Creates a new Loader.
   */

  private CANSparkMax motor;

  public Loader() {
    motor = new CANSparkMax(SparkConstants.MOTOR_LOADER, MotorType.kBrushless);
  }

  public void load() {
    motor.set(1);
  }

  public void release() {
    motor.set(-1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
