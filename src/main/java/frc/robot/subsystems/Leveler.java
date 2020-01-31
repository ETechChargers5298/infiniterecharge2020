/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leveler extends SubsystemBase {
  /**
   * Creates a new Leveler.
   */

   // Holds the Micro NavX for Leveling
   private AHRS navX;

  public Leveler() {
    // Micro NavX Communication with I2C
    try {
      navX = new AHRS(I2C.Port.kMXP);
    }
    catch(RuntimeException ex) {
      DriverStation.reportError("Error instantiating NavX MXP: " + ex.getMessage(), true);
    }
  }

  public void test() {
    SmartDashboard.putNumber("Gyro Micro", navX.getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
