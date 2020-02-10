/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Intake extends SubsystemBase {

  /* INTAKE FIELDS */
  private final DoubleSolenoid intakeSolenoid;
  private final CANSparkMax intakeMotor;

  /* INTAKE CONSTRUCTOR */
  public Intake() {
    // Creating DoubleSolenoid Object for the Intake
    intakeSolenoid = new DoubleSolenoid(DriveConstants.INTAKE_PORT_FIVE, DriveConstants.INTAKE_PORT_SIX);

    intakeMotor = new CANSparkMax(DriveConstants.WHEEL_INTAKE_MOTOR, MotorType.kBrushless);
  }

  /* INTAKE METHODS */
  public void pushIntakePistons() {
    intakeSolenoid.set(Value.kForward);
  }

  public void retractIntakePistons() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void intakeMotorSpeed(final double speed) {
    intakeMotor.set(speed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
