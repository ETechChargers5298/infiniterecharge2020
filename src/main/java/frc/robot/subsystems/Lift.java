/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Lift extends SubsystemBase {
  /**
   * Our Robot has a Elevator Controlled by Pistons. This Allows Us to
   * Exceed the Robots Height Using Lots of Compressed Air to Lift
   * onto the Generator Switch During End Game.
   */

  // Holds Solenoid that Powers Lifts
  private DoubleSolenoid lifterSolenoid;

  /* LIFT CONSTRUCTOR */
  public Lift() {
    // Constructs a Solenoid Object for Lifter
    lifterSolenoid = new DoubleSolenoid(LiftConstants.LIFT_PORT_ZERO,
    LiftConstants.LIFT_PORT_ONE);
  }

  // Elevator Goes Up to Reach for Generator Switch Bar
  public void robotReach(){
    lifterSolenoid.set(Value.kForward);
  }

  // Elevator Goes Down to Pull The Robot on the Bar
  public void robotPull(){
    lifterSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
