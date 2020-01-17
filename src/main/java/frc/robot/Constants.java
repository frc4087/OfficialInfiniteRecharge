/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.HashMap;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Drivebase Motor Controllers
  public static final int LFF = 0;
  public static final int LMF = 1;
  public static final int LMB = 2;
  public static final int LBB = 3;
  public static final int RFF = 4;
  public static final int RMF = 5;
  public static final int RMB = 6;
  public static final int RBB = 7;

  public static final int kShooterAngleMotor = 8;
  public static final int kShooterRPMMotor = 9;

  public static final int kCPMotor = 10;

  public static final int kColorSensor = 11;

  public static final MotorType kMotorType = MotorType.kBrushless;

  public static final int CPR = 8192;//counts per rev

  //Joystick Ports 
  public static final int zero = 0,
                          one = 1;

  //Joystick Axes
  public static final int XL = 0,
                          YL = 1,
                          XR = 4,
                          YR = 5;

  public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
  public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
  public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);

  public static final HashMap<char, Color> FMStoColor = new HashMap<char, Color>();
  FMStoColor.put('R', kBlueTarget);
  FMStoColor.put('G', kYellowTarget);
  FMStoColor.put('B', kRedTarget);
  FMStoColor.put('Y', kGreenTarget);

}
