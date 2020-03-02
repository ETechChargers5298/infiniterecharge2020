/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.experimental.PIDShooter;

public class AutoPIDShoot extends CommandBase {
  /**
   * Creates a new AutoPIDShoot.
   */
  private PIDShooter shooter;

  private int desiredShots;
  private int shotCounter;
  private boolean shotCounted;
  public AutoPIDShoot(PIDShooter shooter, int desiredShots) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.desiredShots = desiredShots;
    this.shotCounter = 0;
    this.shotCounted  = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Turns on Shooter
    shooter.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.atSetpoint()) {
      shooter.load();
      shotCounted = false;
    }
    else {
      if(!shotCounted) {
        shotCounter++;
      }
      shotCounted = true;
      shooter.stopLoading();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.disable();
    shooter.stopLoading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(desiredShots == shotCounter) {
      return true;
    }
    return false;
  }
}
