/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SolenoidConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift extends SubsystemBase {
  /**
   * Our Robot has a Elevator Controlled by Pistons. This Allows Us to
   * Exceed the Robots Height Using Lots of Compressed Air to Lift
   * onto the Generator Switch During End Game.
   */

  // Holds Solenoid that Powers Lifts
  private DoubleSolenoid lifterSolenoid;

  private Boolean isReaching;

  /* LIFT CONSTRUCTOR */
  public Lift() {
    // Constructs a Solenoid Object for Lifter
    lifterSolenoid = new DoubleSolenoid(SolenoidConstants.LIFT_PORT_A,
    SolenoidConstants.LIFT_PORT_B);

    robotPull();

    isReaching = false;
  }

  // Elevator Goes Up to Reach for Generator Switch Bar
  public void robotReach(){
    isReaching = true;
    lifterSolenoid.set(Value.kReverse);
  }

  // Elevator Goes Down to Pull The Robot on the Bar
  public void robotPull(){
    isReaching = false;
    lifterSolenoid.set(Value.kForward);
  }

  public void printData() {
    if(isReaching) {
      SmartDashboard.putString("Lift State", "Reach");
    }
    else {
      SmartDashboard.putString("Lift State", "Climb");
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
