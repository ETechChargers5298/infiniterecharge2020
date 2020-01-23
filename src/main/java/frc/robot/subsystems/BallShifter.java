/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallShifter extends SubsystemBase {
  /**
   * Creates a new BallShifter.
   */

   // Create objects in DoubleSolenoid.
  private DoubleSolenoid shifter = new DoubleSolenoid (Constants.SHIFTER_MODULE,
   Constants.SHIFTER_PORT_ONE, Constants.SHIFTER_PORT_TWO);

  // Starts at High Torque
  private boolean driveMode = false;

  public BallShifter() {

  }

  // Goes into high torque mode using the shifter.

  public void highTorque() {
    shifter.set(Value.kForward);
  }

  // Goes into high speed mode using the shifter.

  public void highSpeed() {
    shifter.set(Value.kReverse);
  }

 public void toggleDriveMode() {
   driveMode = !driveMode;
   if(driveMode) {
     highSpeed();
   }
   else {
     highTorque();
   }
 }

  public boolean getDriveMode(){
    return driveMode;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
