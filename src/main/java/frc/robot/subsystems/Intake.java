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
import frc.robot.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /**
   * Our Robot has the Ability to Drop and Retract its Intake to Ensure
   * that We will Not Exceed Our Robot's Max Perimeter. To Actually Intake Balls,
   * We Use a Motor Connected to Chains Which Spins a Axle Containing Many 2 Inch Mechanum
   * Wheels.
   */

  // Holds Solenoids that Drop and Retract Intake
  private DoubleSolenoid intakeSolenoid;

  // Holds Motor That Moves Intake Axle
  private CANSparkMax intakeMotor;

  public Intake() {
    // Creating DoubleSolenoid Object
    intakeSolenoid = new DoubleSolenoid(IntakeConstants.INTAKE_PORT_ZERO,
    IntakeConstants.INTAKE_PORT_ONE);

    // Creating a Motor Object
    intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, 
    MotorType.kBrushless);

    // Inverts Motor if Needed
    intakeMotor.setInverted(IntakeConstants.INTAKE_MOTOR_INVERSION);

    // The Robot Starts With a Retracted Intake
    retractIntake();
  }
  
  // Pushes Intake Down to Start Collecting Balls
  public void dropIntake() {
    intakeSolenoid.set(Value.kForward);
  }

  // Retracts Intake to Get Within Perimeter
  public void retractIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  // Axle Moves in One Direction to Grab Balls
  public void grabBall(){
    intakeMotor.set(IntakeConstants.INTAKE_MAX_SPEED);
  }

  // Axle Moves in Opposite Direction to Drop Balls
  public void releaseBall() {
    intakeMotor.set(-1 * IntakeConstants.INTAKE_MAX_SPEED);
  }

  // Stops the Intake Motor
  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
