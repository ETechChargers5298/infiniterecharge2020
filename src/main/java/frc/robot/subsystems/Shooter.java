/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SolenoidConstants;
import frc.robot.Constants.SparkConstants;
import frc.robot.utils.LimeLight;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  // Holds Motor That Rotates To Launch Ball
  private CANSparkMax shooterMotor;

  // Holds Encoder to Measure Velocity of Launcher
  private CANEncoder shooterEncoder;

  // Holds Piston That Changes Angle of Shooter
  private DoubleSolenoid shooterSolenoid;

  // Holds LimeLight Which Is Used For Aiming
  private LimeLight lime;

  public Shooter() {
    // Constructs Motor for Shooting
    shooterMotor = new CANSparkMax(SparkConstants.MOTOR_SHOOTER, MotorType.kBrushless);

    // Inverts Motor if Needed
    shooterMotor.setInverted(ShooterConstants.SHOOTER_MOTOR_INVERSION);

    // Obtains Encoder from SparkMax
    shooterEncoder = shooterMotor.getEncoder();

    // Constructs a Solenoid to Change Angles
    shooterSolenoid = new DoubleSolenoid(SolenoidConstants.ANGLER_PORT_A, SolenoidConstants.ANGLER_PORT_B);

    // Constructs a Limelight to Aim
    lime = new LimeLight();
  }
  
  // Shoots at Max Power
  public void shootMaxVelocity() {
    shooterMotor.set(ShooterConstants.SHOOTER_MAX_SPEED);
  }

  // Stops Shooting
  public void stopShooting() {
    shooterMotor.stopMotor();
  }

  // Changes to High Angle
  public void highAngle() {
    shooterSolenoid.set(Value.kForward);
  }

  // Changes to Low Angle
  public void lowAngle() {
    shooterSolenoid.set(Value.kReverse);
  }

  // Prints Data Relating to Shooter
  public void printData() {
    // Prints LimeLight Values for Shooter
    lime.printData();
  }

  /* SHOOTER METHODS */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lime.updateLimeLight();
    lime.printData();
  }
}
