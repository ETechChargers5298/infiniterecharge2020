/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

public class ShooterTimed extends WaitCommand {
  
  /**
   * Creates a new Shoot command.
   */

  private Shooter shooter;
  private Intake intake;
  
  public ShooterTimed(Shooter shooter, Intake intake, double shootTime) {
    super(shootTime);

    this.shooter = shooter;
    this.intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.shooter);
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    shooter.setShooterVolts(11);
    intake.eatBall();
    intake.retractIntake();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int velocity = (int) shooter.getShooterVelocity();
    int maximum = (int) (ShooterConstants.SHOOTER_TARGET_RPM + ShooterConstants.SHOOTER_TOLERANCE_RPM);
    int minimum = (int) (ShooterConstants.SHOOTER_TARGET_RPM - ShooterConstants.SHOOTER_TOLERANCE_RPM);

    SmartDashboard.putNumber("Maximum", ShooterConstants.SHOOTER_TARGET_RPM + ShooterConstants.SHOOTER_TOLERANCE_RPM);
    SmartDashboard.putNumber("Minimum", ShooterConstants.SHOOTER_TARGET_RPM - ShooterConstants.SHOOTER_TOLERANCE_RPM);
    SmartDashboard.putNumber("Velocity", shooter.getShooterVelocity());
    SmartDashboard.putNumber("Velocity End", shooter.getShooterVelocity());
    
    shooter.checkShot();
    SmartDashboard.putNumber("Shot Count", shooter.shotCount());


    if(velocity < maximum && velocity > minimum) {
      shooter.load();
    }
    else {
      shooter.stopLoading();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.setShooterVolts(0);
    shooter.stopLoading();
    intake.stopIntake();
    intake.chompIntake();
  }

  // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//       return false;
//   }
 }
