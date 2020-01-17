/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensorBase extends SubsystemBase {
  
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.kColorSensor);
  public final CANSparkMax CPMotor = new CANSparkMax(Constants.kCPMotor, Constants.kMotorType);

  public CMPBase() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
