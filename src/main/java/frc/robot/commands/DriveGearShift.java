/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.RobotContainer;

public class DriveGearShift extends CommandBase {
  /**
   * Creates a new GearShift.
   */

  // Holds the DriveTrain Subsystem
  private final DriveTrain driveTrain;
  
  public DriveGearShift() {
    // Passes the DriveTrain Subsystem Into the Field
    this.driveTrain = RobotContainer.driveTrain;

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
    // Toggles Between High Speed and High Torque
    driveTrain.toggleDriveMode();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ends After Executing Once
    return true;
  }
}
