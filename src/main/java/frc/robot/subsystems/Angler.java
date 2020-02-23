/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SparkConstants;

public class Angler extends SubsystemBase {
  /**
   * Creates a new Angler.
   */

  // Holds Motor to Angle the Shooter
  private final CANSparkMax anglerMotor;

  // Holds Encoder to Measure the Angle of the Angler
  private final CANEncoder anglerEncoder;

  private DigitalInput limitHighAngle;

  private DigitalInput limitLowAngle;
  
  public Angler() {
    // Constructs Motor for Shooting
    anglerMotor = new CANSparkMax(SparkConstants.MOTOR_ANGLER, MotorType.kBrushed);

    // Inverts Motor if Needed
    anglerMotor.setInverted(ShooterConstants.ANGLER_MOTOR_INVERSION);

    // Obtains Angler Encoder from SparkMax
    anglerEncoder = anglerMotor.getEncoder(EncoderType.kQuadrature, 8192);

    zeroAngleRaw();

    limitHighAngle = new DigitalInput(0);

    limitLowAngle = new DigitalInput(1);
  }

  // Change angle of Angler Motor Manually
  public void moveAngle(double speed) {
    anglerMotor.set(speed);
  }

  public int getAngleRaw() {
    return (int)(anglerEncoder.getPosition() * 1000);
  }

  public boolean getHighLimit() {
    return limitHighAngle.get();
  }

  public boolean getLowLimit() {
    return limitLowAngle.get();
  }

  public void zeroAngleRaw() {
    anglerEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Angler Encoder Data", getAngleRaw());

    if(!getHighLimit()) {
      zeroAngleRaw();
    }
  }
}
