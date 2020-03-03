/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.experimental;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SparkConstants;

public class PIDShooter extends PIDSubsystem {
  /**
   * Creates a new NewPIDShooter.
   */

  private CANSparkMax shooterMotor;

  private CANEncoder shooterEncoder;

  private CANSparkMax loaderMotor;

  private SimpleMotorFeedforward feedforward;

  private Boolean loaderOn;
  private Boolean loaderLoading;

  public PIDShooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D));

        // Creates Shooter Motor Object
        shooterMotor = new CANSparkMax(SparkConstants.MOTOR_SHOOTER, MotorType.kBrushless);

        shooterMotor.setInverted(ShooterConstants.SHOOTER_MOTOR_INVERSION);

        // Obtains the Shooter Motor Encoder
        shooterEncoder = shooterMotor.getEncoder();

        // Creates Loader Motor Object
        loaderMotor = new CANSparkMax(SparkConstants.MOTOR_LOADER, MotorType.kBrushless);

        loaderMotor.setInverted(ShooterConstants.LOADER_MOTOR_INVERSION);

        // Creates a Feedforward for the Shooter Motor
        feedforward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_VOLTS, ShooterConstants.SHOOTER_VOLTS_SECONDS_PER_ROTATION);

        // Sets Tolerance
        getController().setTolerance(ShooterConstants.SHOOTER_TOLERANCE_RPM);

        // Sets Setpoint of Shooter
        setSetpoint(ShooterConstants.SHOOTER_TARGET_RPM);

        // Loader is Off at Start
        loaderOn = false;
        loaderLoading = false;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    shooterMotor.setVoltage(output + feedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
    return shooterEncoder.getVelocity();
  }

  public boolean atSetpoint() {
    // Obtains Controller from Super Field to Check
    return m_controller.atSetpoint();
  }

  public void load() {
    loaderOn = true;
    loaderLoading = true;
    loaderMotor.set(ShooterConstants.LOAD_SPEED);
  }

  public void unload() {
    loaderOn = true;
    loaderLoading = false;
    loaderMotor.set(-1 * ShooterConstants.LOAD_SPEED);
  }

  public void stopLoading() {
    loaderOn = false;
    loaderLoading = false;
    loaderMotor.set(0);
  }
  
  public void printData() {
    SmartDashboard.putNumber("Shooter Velocity", getMeasurement());
    SmartDashboard.putBoolean("Shooter Setpoint", atSetpoint());
    SmartDashboard.putBoolean("Loader On", loaderOn);
    SmartDashboard.putBoolean("Loader Loading", loaderLoading);
  }
}
