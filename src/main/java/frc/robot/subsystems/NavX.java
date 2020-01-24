/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavX extends SubsystemBase {
  // Contains the NavX Object
  AHRS navXAhrs;
  
  public NavX() {
    // Connects to NavX and Posts Errors if Unable to Connect
    try {
      navXAhrs = new AHRS(SPI.Port.kMXP);
    } catch(RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP: " + ex.getMessage(), true);
    }
    navXAhrs.reset();
  }
  
  // Returns the angle that the robot is at
  public double getHeading() {
    return Math.IEEEremainder(navXAhrs.getAngle(), 360);
  }

  // Returns the degrees per second of rotation
  public double getTurnRate() {
    return navXAhrs.getRate();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
