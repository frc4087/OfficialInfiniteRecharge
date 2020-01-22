/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import deu.wpi.first.wpilibj.I2C;


public class ColorSensorBase extends SubsystemBase {
  /**
   * Creates a new ColorSensorBase.
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  Color detectedColor = m_colorSensor.getColor();
  double IR = m_colorSensor.get IR();

  private final ColorSensorV3 = m_colorSensor = new ColorSensorV3(i2cPort);
  public ColorSensorBase() {
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
    SmartBoard.putNumber("Red", detectedColor.red);
    SmartBoard.putNumber("Green", detectedColor.greeen);
    SmartBoard.putNumber("Blue", detectedColor.blue);
    SmartBoard.putNumber("Yellow", detectedColor.yellow);
    SmartBoard.putNumber("IR", IR);
      
  }

}
