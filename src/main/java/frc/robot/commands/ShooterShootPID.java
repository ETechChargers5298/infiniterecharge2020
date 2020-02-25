/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.experimental.PIDShooter;

public class ShooterShootPID extends CommandBase {
  /**
   * Creates a new ShooterShootPID.
   */

  private PIDShooter shooter;

  public ShooterShootPID(PIDShooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Turns on Shooter PID
    shooter.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.atSetpoint()) {
      shooter.load();
    }
    else {
      shooter.stopLoading();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops Shooting and Loading
    shooter.disable();
    shooter.stopLoading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
