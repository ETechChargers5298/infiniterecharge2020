/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootPID extends PIDCommand {
  /**
   * Creates a new ShootPID.
   */
  private Shooter shooter;
  public ShootPID(Shooter shooter) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0),
        // This should return the measurement
        shooter::getShooterVelocity,
        // This should return the setpoint (can also be a constant)
        ShooterConstants.SHOOTER_TARGET_RPM,
        // This uses the output
        output -> shooter.setShooterVolts(output + ShooterConstants.SHOOTER_VOLTS_SECONDS_PER_ROTATION * ShooterConstants.SHOOTER_TARGET_RPM),
        // Subsystems that are used
        shooter
        );

        this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(100);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
