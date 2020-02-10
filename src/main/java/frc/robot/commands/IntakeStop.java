/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD:src/main/java/frc/robot/commands/StopIntake.java
import frc.robot.subsystems.Intake;

public class StopIntake extends CommandBase {
=======
import frc.robot.RobotContainer;

public class IntakeRetractUp extends CommandBase {
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c:src/main/java/frc/robot/commands/IntakeRetractUp.java
  /**
   * Creates a new StopIntake.
   */
<<<<<<< HEAD:src/main/java/frc/robot/commands/StopIntake.java

   // Holds the Intake Subsystem
   private Intake intake;
   
  public StopIntake(Intake intake) {
    // Passes the Intake Subsystem into Field
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
=======
  public IntakeRetractUp() {
      addRequirements(RobotContainer.intake);
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c:src/main/java/frc/robot/commands/IntakeRetractUp.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/StopIntake.java
=======
    RobotContainer.intake.retractIntakePistons();
>>>>>>> 340802fa3a435c07d2a71c67f4fe66145c7a405c:src/main/java/frc/robot/commands/IntakeRetractUp.java
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Stops The Intake Mechanism
    intake.stopIntake();
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
