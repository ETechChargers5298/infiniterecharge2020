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
   * Creates a new Shooter.
   */

  private CANSparkMax shooterMotor;
  private CANEncoder shooterEncoder;
  private SimpleMotorFeedforward shooterFeedforward;
  private CANSparkMax loaderMotor;

  public PIDShooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D));

        // Creates a Shooter Motor Object
        shooterMotor = new CANSparkMax(SparkConstants.MOTOR_SHOOTER, MotorType.kBrushless);

        // Inverts Motor
        shooterMotor.setInverted(ShooterConstants.SHOOTER_MOTOR_INVERSION);

        // Obtains Encoder Connected to the Shooter Motor SparkMax
        shooterEncoder = shooterMotor.getEncoder();

        // Includes Feedforward for Shooter
        shooterFeedforward = new SimpleMotorFeedforward(ShooterConstants.SHOOTER_VOLTS, ShooterConstants.SHOOTER_VOLTS_SECONDS_PER_ROTATION);

        // Contains the Motor Controlling Loading
        loaderMotor = new CANSparkMax(SparkConstants.MOTOR_LOADER, MotorType.kBrushless);

        getController().setSetpoint(ShooterConstants.SHOOTER_TARGET_RPM);

        //getController().setTolerance(ShooterConstants.SHOOTER_TOLERANCE_RPM);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    shooterMotor.setVoltage(output+ shooterFeedforward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return shooterEncoder.getVelocity();
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  public void stopShooter() {
    shooterMotor.setVoltage(0);
  }

  public void load() {
    loaderMotor.set(1);
  }

  public void stopLoading() {
    loaderMotor.set(0);
  }
}
