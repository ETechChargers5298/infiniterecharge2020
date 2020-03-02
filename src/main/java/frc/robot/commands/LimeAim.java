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

public class LimeAim extends CommandBase {
  /**
   * Creates a new LimeAim.
   */

  private DriveTrain driveTrain;
  private LimeLight limelight;
  
  // Holds Old Speed
  private double leftSpeed;
  private double rightSpeed;

  public LimeAim(DriveTrain driveTrain, LimeLight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.limelight = limelight;

    leftSpeed = 0;
    rightSpeed = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Updates Limelight Values
    limelight.updateLimeLight();

    // Uses Limelight Data
    double headingError = -limelight.getX();
    double distanceError = -limelight.getY();
    double steeringAdjust = LimeLightConstants.AIM_P * headingError;

    // Calculates Turn Based on How Close To Target We Are
    if(limelight.getX() > 1) {
      steeringAdjust -= LimeLightConstants.AIM_MIN_SPEED;
    }
    else {
      steeringAdjust += LimeLightConstants.AIM_MIN_SPEED;
    }

    // Calculates if We Are At Desired Distance
    double distanceAdjust = LimeLightConstants.DISTANCE_P * distanceError;

    leftSpeed += steeringAdjust + distanceAdjust;
    rightSpeed -= steeringAdjust + distanceAdjust;

    driveTrain.powerDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Makes Sure DriveTrain has Ended
    driveTrain.powerDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
