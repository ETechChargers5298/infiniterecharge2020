/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.LimeLight;

public class LimeAlignToShoot extends CommandBase {
  /**
   * Creates a new LimeAlignToShoot.
   */

  private DriveTrain driveTrain;
  private LimeLight limelight;

  private double desiredX;
  private double desiredY;

  private double driveSpeed;
  private double steerSpeed;

  public LimeAlignToShoot(DriveTrain driveTrain, LimeLight limeLight, double x, double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limelight = limelight;
    this.desiredX = x;
    this.desiredY = y;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Calibrates X and Y
    limelight.calibrate(desiredX, desiredY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Constantly Updates Data
    limelight.updateLimeLight();

    if(!limelight.hasValidTarget()) {
      driveSpeed = 0;
      steerSpeed = 0;
    }

    steerSpeed = limelight.getX() * LimeLightConstants.STEER_P;
    driveSpeed = limelight.getY() * LimeLightConstants.DRIVE_P;

    // Makes Sure Its Drive is Not Too Fast
    if(driveSpeed > LimeLightConstants.MAX_SPEED) {
      driveSpeed = LimeLightConstants.MAX_SPEED;
    }

    driveTrain.arcadeDrive(driveSpeed, steerSpeed);
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
