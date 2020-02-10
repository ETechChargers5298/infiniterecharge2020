/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD:src/main/java/frc/robot/commands/GrabBall.java
import frc.robot.subsystems.Intake;

public class GrabBall extends CommandBase {
=======
import frc.robot.RobotContainer;

public class LiftReach extends CommandBase {
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c:src/main/java/frc/robot/commands/LiftReach.java
  /**
   * Creates a new GrabBall.
   */
<<<<<<< HEAD:src/main/java/frc/robot/commands/GrabBall.java

  // Holds the Intake Subsystem
  private Intake intake;

  public GrabBall(Intake intake) {
    // Passes the Intake Subsystem into Field
    this.intake = intake;

=======
  public LiftReach() {
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c:src/main/java/frc/robot/commands/LiftReach.java
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Moves Motors to Intake the Balls
    intake.grabBall();
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
