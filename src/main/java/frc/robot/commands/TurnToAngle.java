/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.robot.Constants.TurnToAngleConstants;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /**
   * Creates a new TurnToAngle.
   */

  // Holds the DriveTrain Subsystem
  private final DriveTrain driveTrain;

  public TurnToAngle(double targetAngleDegrees, DriveTrain driveTrain) {
    super(
        // The Controller that the Command will Use
        new PIDController(TurnToAngleConstants.TURN_P, TurnToAngleConstants.TURN_I, TurnToAngleConstants.TURN_D),
        // This should Return the Measurement
        driveTrain::getHeading,
        // This Should Return the Setpoint
        targetAngleDegrees,
        // This uses the Output
        output -> driveTrain.driveSpeed(output, output),
        // Subsystems that are Used
        driveTrain
        );
    
    // Updates Subsystem Fields
    this.driveTrain = driveTrain;

    // These are Required Subsystems
    addRequirements(this.driveTrain);

    // Instead of Min and Max, PID Knows its can Turn Infinitely
    getController().enableContinuousInput(-180, 180);

    // Sets Tolerance
    getController().setTolerance(TurnToAngleConstants.TURN_TOLERANCE, TurnToAngleConstants.TURN_RATE_TOLERANCE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ends When the Robot is Facing the Target Angle
    return getController().atSetpoint();
  }
}
