/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends PIDCommand {
  /**
   * Creates a new TurnToAngle.
   */

  // Holds subsytems
  DriveTrain drive;
  NavX gyro;
  public TurnToAngle(double targetAngleDegrees, DriveTrain drive, NavX gyro) {
    super(
        // The controller that the command will use
        new PIDController(Constants.TURN_P, Constants.TURN_I, Constants.TURN_D),
        // This should return the measurement
        gyro::getHeading,
        // This should return the setpoint (can also be a constant)
        targetAngleDegrees,
        // This uses the output
        output -> drive.driveSpeed(output, output),
        // Subsystems that are used
        drive, gyro);
    
    // Updates subsystem fields
    this.drive = drive;
    this.gyro = gyro;

    // These are required subsystems
    addRequirements(this.drive, this.gyro);

    // Instead of Min and Max, PID knows its can turn infinitely
    getController().enableContinuousInput(-180, 180);
    // Sets Tolerance
    getController().setTolerance(Constants.TURN_TOLERANCE, Constants.TURN_RATE_TOLERANCE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Ends when the controller is at the angle
    return getController().atSetpoint();
  }
}
