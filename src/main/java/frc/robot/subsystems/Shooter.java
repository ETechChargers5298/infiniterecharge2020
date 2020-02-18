/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
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

  //Holds Motor to Angle the Shooter
  private CANSparkMax anglerMotor;

  //Holds Encoder to Measure the Angle of the Angler
  private CANEncoder anglerEncoder;

  //
  private CANSparkMax loaderMotor;

  // Holds LimeLight Which Is Used For Aiming
  private LimeLight lime;

  public Shooter() {
    // Constructs Motor for Shooting
    shooterMotor = new CANSparkMax(SparkConstants.MOTOR_SHOOTER, MotorType.kBrushless);

    // Inverts Motor if Needed
    shooterMotor.setInverted(ShooterConstants.SHOOTER_MOTOR_INVERSION);

    // Obtains Encoder from SparkMax
    //shooterEncoder = shooterMotor.getEncoder();

    // Constructs Motor for Shooting
    anglerMotor = new CANSparkMax(SparkConstants.MOTOR_ANGLER, MotorType.kBrushed);

    // Inverts Motor if Needed
    anglerMotor.setInverted(ShooterConstants.ANGLER_MOTOR_INVERSION);

    // Obtains Angler Encoder from SparkMax
    anglerEncoder = anglerMotor.getEncoder(EncoderType.kQuadrature, 8192);

    loaderMotor = new CANSparkMax(SparkConstants.MOTOR_LOADER, MotorType.kBrushless);

    // Constructs a Limelight to Aim
    lime = RobotContainer.limeLight;
  }
  
  // Shoots at Max Power
  public void shootMaxVelocity() {
    shooterMotor.set(ShooterConstants.SHOOTER_MAX_SPEED);
  }

  // Stops Shooting
  public void stopShooting() {
    shooterMotor.set(0.0);
  }
  
  // Does the Loader
  public void load() {
    loaderMotor.set(ShooterConstants.LOAD_SPEED);
  }

  //Stops the loader
  public void stopLoading(){
    loaderMotor.set(0.0);
  }



  // Change angle of Angler Motor Manually
  /* METHOD NEED LIMIT SWITCHES ADDED TO PREVENT BREAKING!!!! */
  public void moveAngle(double speed) {
    anglerMotor.set(speed);
  }

  // Change angle of Angler Motor with a PID Loop and Encoder
  /* METHOD NEED LIMIT SWITCHES ADDED TO PREVENT BREAKING!!!! */
  public void autoAngle(double shotAngle) {
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
    SmartDashboard.putNumber("Angler Encoder Data", anglerEncoder.getPosition());
  }
}
