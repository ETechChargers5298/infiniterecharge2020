/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;


public class AutoDriveStraight extends CommandBase {
  /**
   * Creates a new AutonomousDriveStraight.
   */
  
  private DriveTrain drive;
  private double speed;
  private double maxTime;
  private double startTime;
  private boolean finished;

  public AutoDriveStraight(DriveTrain drive, double speed, double timer) {
    this.drive = drive;
    this.speed = speed;
    this.maxTime = timer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.arcadeDrive(speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - startTime >= maxTime) {
        finished = true;
    }
    
    return finished;
  }
}
