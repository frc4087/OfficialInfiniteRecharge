/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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
  public static final int LF = 0,
                          LB = 1,
                          RF = 2,
                          RB = 3;

  //Joystick Ports 
  public static final int zero = 0,
                          one = 1;

  //Joystick Axes
  public static final int XL = 0,
                          YL = 1,
                          XR = 4,
                          YR = 5;

  //Characterization Toolsuite Constants
  public static final double ksVolts = 0.546,
                             kvVoltSecondsPerMeter = 0.083,
                             kaVoltSecondsSquaredPerMeter = 0.00889,
                             kTrackwidthMeters = (21.75*2.54)/100,
                             kMaxSpeedMetersPerSecond = 3, //Need to be changed
                             kMaxAccelerationMetersPerSecondSquared = 3, //Need to be changed
                             kRamseteB = 2,
                             kRamseteZeta = 0.7,
                             kPDriveVel = 0.2; //CHANGE THIS LATER
                             
  public static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

  //Encoder Constants
  public static final double kEncoderDistancePerPulse = (6*3.14)/(360*4);

  public static final boolean kGyroReversed = false;
}
