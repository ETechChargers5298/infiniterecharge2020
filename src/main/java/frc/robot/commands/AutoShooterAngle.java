/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShooterAngle extends PIDCommand {
  /**
   * Creates a new ShooterAngle.
   */

  public AutoShooterAngle(Angler angler, double targetRawAngle) {
    super(
        // The controller that the command will use
        new PIDController(0.005, 0, 0),
        // This should return the measurement
        angler::getAngleRaw,
        // This should return the setpoint (can also be a constant)
        targetRawAngle,
        // This uses the output
        output -> RobotContainer.angler.moveAngle(-output),
          // Use the output here
          RobotContainer.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(10);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
