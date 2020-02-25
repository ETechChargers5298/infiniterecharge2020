/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.LimeLight;

public class DriveAimRange extends CommandBase {
  /**
   * Creates a new DriveAimRange.
   */

  private DriveTrain driveTrain;
  private LimeLight limelight;

  private double tx;
  private double ty;
  private double targetX;
  private double targetY;
  
  public DriveAimRange(DriveTrain driveTrain, LimeLight lime) {
    this.driveTrain = driveTrain;
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updating tx and ty
    tx = limelight.getX();
    ty = limelight.getY();

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
