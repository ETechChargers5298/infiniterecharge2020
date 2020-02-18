/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShooterLoadOnly extends CommandBase {
  
  Shooter shooter;
  
  public ShooterLoadOnly(Shooter shooter) {

    this.shooter = shooter;
    addRequirements(this.shooter);        // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.load();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopLoading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(RobotContainer.operatorController.getTriggerAxis(GenericHID.Hand.kRight) <= 0.5) {
    //   return true;
    // }
    return false;
  }
}
