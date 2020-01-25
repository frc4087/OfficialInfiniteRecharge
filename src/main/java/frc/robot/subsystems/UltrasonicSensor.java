/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicSensor extends SubsystemBase {
  /**
   * Creates a new UltrasonicSensor.
   */
  //A supply of 5V yields ~4.9mV/cm

  public final AnalogInput m_ultra = new AnalogInput(0);
  
  double currentDistance;
  
  public double getDistance(){
    return (m_ultra.getValue()/4.9)*2.54;
  }

  public UltrasonicSensor() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
