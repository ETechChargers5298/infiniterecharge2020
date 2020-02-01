/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Lift extends SubsystemBase {
  private DoubleSolenoid lifterSolenoid;


  public Lift() {
    
    lifterSolenoid = new DoubleSolenoid(DriveConstants.LIFTER_PORT_THREE, 
    DriveConstants.LIFTER_PORT_FOUR);
  }

  public void forwardSolenoid(){
    lifterSolenoid.set(Value.kForward);
    
  }

  public void reverseSolenoid(){
    lifterSolenoid.set(Value.kReverse);

  }

  public void offSolenoid(){
    lifterSolenoid.set(Value.kOff);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
