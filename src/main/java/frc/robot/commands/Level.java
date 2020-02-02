/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.robot.Constants.LevelConstants;
import frc.robot.subsystems.Leveler;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Level extends ProfiledPIDCommand {
  /**
   * Creates a new Leveler.
   */

  // Holds the Leveler Subsystem
  private final Leveler leveler;
  public Level(Leveler leveler) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            LevelConstants.LEVEL_P, LevelConstants.LEVEL_I, LevelConstants.LEVEL_D,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(LevelConstants.MAX_VELOCITY, LevelConstants.MAX_ACCELERATION)),
        // This should return the measurement
        leveler::getRoll,
        // This should return the goal
        LevelConstants.GOAL,
        // This uses the output
        (output, setpoint) -> 
        // Use the output (and setpoint, if desired) here
        leveler.move(output),
        // Required Subsystems
        leveler
        );

    // Updates Leveler Field
    this.leveler = leveler;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.leveler);

    // Sets Tolerance For Setpoint to Know What Speed it Has to Be At
    getController().setTolerance(LevelConstants.DEGREE_TOLERANCE, LevelConstants.VELOCITY_TOLERANCE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ends When Robot is Leveled
    return getController().atGoal();
  }
}
