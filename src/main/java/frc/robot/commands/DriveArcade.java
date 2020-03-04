/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OldDriveTrain;
import frc.robot.subsystems.DriveTrain;


public class DriveArcade extends CommandBase {
  /**
   * Creates a new ArcadeDrive.
   */
  
  // Holds the DriveTrain Subsystem
  private DriveTrain driveTrain;

  // Holds the Supplier of the Velocities
  private final DoubleSupplier linVelocity;
  private final DoubleSupplier rotVelocity;

  public DriveArcade(DriveTrain driveTrain, DoubleSupplier linVelocity, DoubleSupplier rotVelocity) {
    // Passes the DriveTrain Subsystem Into the Field
    this.driveTrain = driveTrain;

    // Uses Joystick as Our Velocity Supplier
    this.linVelocity = linVelocity;
    this.rotVelocity = rotVelocity;

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
    // Uses Joystick Velocity to Implement Arcade Drive
    driveTrain.slewArcadeDrive(linVelocity.getAsDouble(), -rotVelocity.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stops Driving Completely
    driveTrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This Command Never Ends
    return false;
  }
}
