/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.OldDriveTrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDriveStraight extends PIDCommand {
  /**
   * Creates a new AutonomousDriveStraight.
   */
  
  private DriveTrain driveTrain;
  private double speed;
  private double inchDistance;
  private double startDistance;
  private double rawDistance;
  private boolean finished;

  public AutoDriveStraight(DriveTrain driveTrain, double speed, double distanceMeters) {
    super(
        // The controller that the command will use
        new PIDController(0.05, 0, 0),
        // This should return the measurement
        driveTrain::getRightPosition,
        // This should return the setpoint (can also be a constant)
        distanceMeters,
        // This uses the output
        output -> driveTrain.powerDrive(MathUtil.clamp(output, -speed, speed), -1 * MathUtil.clamp(output, -speed, speed)),
          // Use the output here
          driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1.0);

    this.driveTrain = driveTrain;
    this.speed = -speed;
    this.inchDistance = distanceMeters;
    driveTrain.resetRightPosition();

    SmartDashboard.putNumber("Target Meters", distanceMeters);


    // These are Required Subsystems
    addRequirements(RobotContainer.driveTrain);


  }

 

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  
  }
}
