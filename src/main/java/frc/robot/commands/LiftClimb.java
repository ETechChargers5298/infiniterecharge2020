/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lift;

public class LiftClimb extends CommandBase {
  /**
   * Creates a new PullUpBar.
   */

  // Holds the Lift Subsystem
  private Lift lift;

  public LiftClimb(Lift lift) {
    // Passes the Lift Subsystem into Field
    this.lift = lift;

    // Use addRequirements() here to declare subsystem dependencies.\
    addRequirements(this.lift);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Robot Pulls Up On Generator Switch
    lift.robotPull();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Runs Only Once
    return true;
  }
}
