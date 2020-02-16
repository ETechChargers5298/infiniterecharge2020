/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Leveler;

public class MoveLevel extends CommandBase {
  /**
   * Creates a new MoveLevel.
   */

   private Leveler leveler;
   private DoubleSupplier speed;
   
  public MoveLevel(Leveler leveler, DoubleSupplier speed) {

    this.leveler = leveler;
    this.speed = speed;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.leveler);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leveler.move(speed.getAsDouble());
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
