/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.experimental.PIDShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShooterGroupLoadShoot extends ParallelCommandGroup {
  /**
   * Creates a new ShooterGroupLoadShoot.
   */
  public ShooterGroupLoadShoot(PIDShooter shooter) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
    super(
      new InstantCommand(shooter::enable, shooter),
      new ConditionalCommand(
        new InstantCommand(shooter::load, shooter), 
        new InstantCommand(), 
        shooter::atSetpoint
      )
    );
  }
}
