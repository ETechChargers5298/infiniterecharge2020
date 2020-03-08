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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OldDriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.AutoDriveStraight;
import frc.robot.commands.DriveShiftSpeed;
import frc.robot.commands.DriveMetersReset;
import frc.robot.commands.IntakeChomp;
import frc.robot.commands.LiftClimb;

public class AutoTripleTrench extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */

   private final Shooter shooter;
  private final DriveTrain driveTrain; // Holds DriveTrain Subsystem
  private final Intake intake;

  public AutoTripleTrench(Shooter shooter, DriveTrain driveTrain,Intake intake) {

    this.shooter = shooter;
    this.driveTrain = driveTrain;
    this.intake = intake;

    // Add Commands here:
    //addCommands(new DriveHighSpeed());
    //addCommands(new LiftClimb(RobotContainer.lift));

    addCommands(
      new AutoTripleShot(shooter, intake),
      new AutoDriveEatTrench(driveTrain, intake)
          
    );
   // addCommands(new IntakeChomp(RobotContainer.intake));

    
  }

  //public final void addCommands​(Command... commands)



}
