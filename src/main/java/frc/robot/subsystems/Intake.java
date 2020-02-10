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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
import frc.robot.robot.Constants.IntakeConstants;
=======
import frc.robot.Constants.DriveConstants;
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c

public class Intake extends SubsystemBase {
  /**
   * Our Robot has the Ability to Drop and Retract its Intake to Ensure
   * that We will Not Exceed Our Robot's Max Perimeter. To Actually Intake Balls,
   * We Use a Motor Connected to Chains Which Spins a Axle Containing Many 2 Inch Mechanum
   * Wheels.
   */

<<<<<<< HEAD
  // Holds Solenoids that Drop and Retract Intake
  private DoubleSolenoid intakeSolenoid;

  // Holds Motor That Moves Intake Axle
  private CANSparkMax intakeMotor;

  // Holds States of Intake
  private boolean isRetracted;
  private boolean isGrabing;
  private boolean isOn;
=======
  /* INTAKE FIELDS */
  private final DoubleSolenoid intakeSolenoid;
  private final CANSparkMax intakeMotor;
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c

  /* INTAKE CONSTRUCTOR */
  public Intake() {
<<<<<<< HEAD
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

    // The Robot Intake Starts Off
    stopIntake();
  }
  
  // Pushes Intake Down to Start Collecting Balls
  public void dropIntake() {
    // Controls Solenoids
=======
    // Creating DoubleSolenoid Object for the Intake
    intakeSolenoid = new DoubleSolenoid(DriveConstants.INTAKE_PORT_FIVE, DriveConstants.INTAKE_PORT_SIX);

    intakeMotor = new CANSparkMax(DriveConstants.WHEEL_INTAKE_MOTOR, MotorType.kBrushless);
  }

  /* INTAKE METHODS */
  public void pushIntakePistons() {
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c
    intakeSolenoid.set(Value.kForward);

    // Updates Robot State
    isRetracted = false;
  }

<<<<<<< HEAD
  // Retracts Intake to Get Within Perimeter
  public void retractIntake() {
    // Controls Solenoids
=======
  public void retractIntakePistons() {
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c
    intakeSolenoid.set(Value.kReverse);

    // Updates Robot State
    isRetracted = true;
  }

<<<<<<< HEAD
  // Axle Moves in One Direction to Grab Balls
  public void grabBall() {
    // Moves Motor
    intakeMotor.set(IntakeConstants.INTAKE_MAX_SPEED);

    // Updates Robot State
    isGrabing = true;
    isOn = true;
=======
  public void intakeMotorSpeed(final double speed) {
    intakeMotor.set(speed);
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c
  }

  // Axle Moves in Opposite Direction to Drop Balls
  public void releaseBall() {
    // Moves Motor
    intakeMotor.set(-1 * IntakeConstants.INTAKE_MAX_SPEED);

    // Updates Robot State
    isGrabing = false;
    isOn = true;
  }

  // Stops the Intake Motor
  public void stopIntake() {
    // Stops Motor
    intakeMotor.stopMotor();

    // Updates Robot State
    isOn = false;
  }

  // Prints Out Data Related to Intake
  public void printData() {
    // The Position of our Intake due to Solenoids
    if(isRetracted) {
      SmartDashboard.putString("Intake Position", "Retracted");
    }
    else {
      SmartDashboard.putString("Intake Position", "Dropped");
    }

    // The Motor Mode for Balls
    SmartDashboard.putBoolean("Intake On", isOn);
    SmartDashboard.putBoolean("Grabbing PowerCell", isGrabing);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
