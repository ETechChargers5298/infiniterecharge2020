/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.RobotContainer;
import frc.robot.utils.LimeLight;
import frc.robot.subsystems.DriveTrain;

public class DriveAimToShoot extends CommandBase {

  private LimeLight limeLight;  // Holds LimeLight Object
  private DriveTrain driveTrain;  // Holds DriveTrain Subsystem

  // Holds Error Fields
  private double headingError;
  private double steeringAdjust;

  // Holds Wheel Speed Commands
  private double leftCommand;
  private double rightCommand;


 //Constructor
  public DriveAimToShoot() {
    // Updates LimeLight field
    this.limeLight = RobotContainer.limeLight;

    // Updates Subsystem Fields
    this.driveTrain = RobotContainer.driveTrain;

    // Commands are Zero due to the First Time Command is Called
    leftCommand = 0;
    rightCommand = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Locates Errors and Makes Adjustments
    headingError = -1 * limeLight.getX();
    steeringAdjust = LimeLightConstants.AIM_P * headingError ;

    // Steering Adjust Compensates for if Target Found
    if(limeLight.hasValidTarget()) {
      steeringAdjust -= LimeLightConstants.AIM_MIN_COMMAND;
    }
    else {
      steeringAdjust += LimeLightConstants.AIM_MIN_COMMAND;
    }
    
    // Updates Wheel Commands
    leftCommand += steeringAdjust;
    rightCommand -= steeringAdjust;

    // Uses Command to Drive Robot
    driveTrain.driveSpeed(leftCommand, rightCommand);
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
