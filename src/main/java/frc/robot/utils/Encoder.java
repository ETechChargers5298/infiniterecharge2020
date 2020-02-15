/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* What advantage is there in having an Encoder class if the API provides accessors already? -Mr. Bianchi */

package frc.robot.utils;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

public class Encoder {
  /**
   * Creates a new Encoders.
   */
  private CANEncoder encoder;
  public Encoder(CANSparkMax motor) {
    encoder = motor.getEncoder();
  }

  public double getDistance() {

    return 0.0;
    
  }

}
