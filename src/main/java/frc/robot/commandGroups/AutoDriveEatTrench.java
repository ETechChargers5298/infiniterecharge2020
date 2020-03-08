/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OldDriveTrain;
import frc.robot.commands.AutoDriveStraight;
import frc.robot.commands.DriveShiftSpeed;
import frc.robot.commands.DriveMetersReset;
import frc.robot.commands.IntakeChomp;
import frc.robot.commands.IntakeEat;
import frc.robot.commands.LiftClimb;

public class AutoDriveEatTrench extends ParallelRaceGroup {
  /**
   * Add your docs here.
   */

  private final DriveTrain driveTrain; // Holds DriveTrain Subsystem
  private final Intake intake;

  public AutoDriveEatTrench(DriveTrain driveTrain,Intake intake) {

    this.driveTrain = driveTrain;
    this.intake = intake;

    // Add Commands here:
    addCommands(new AutoDriveOnly(driveTrain));
    addCommands(new IntakeEat(intake));
    
  }

  //public final void addCommands​(Command... commands)



}
