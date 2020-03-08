/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.OldDriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoTripleShot extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */

 
  private final Shooter shooter; //Holds Shooter subsystem
  private final Intake intake; 

  public AutoTripleShot(Shooter shooter, Intake intake) {

    this.shooter = shooter;
    this.intake = intake;

    // Add Commands here:
   // addCommands(new AutoDriveStraight(driveTrain, 0.5, 2));
    addCommands(
      new ShooterTimed(shooter, intake, 5.0)
    );
    
  }

  //public final void addCommands​(Command... commands)



}
