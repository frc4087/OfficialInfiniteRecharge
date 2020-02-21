/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

public final class Constants {

    // Max Current Limit
    public final static int LIM = 20;

    // DRIVEBASE CONSTANTS
    // Motor Controllers
    public final static int LF = 0;// 4;
    public final static int LB = 1;// 5;
    public final static int RF = 2;
    public final static int RB = 3;// 3;
    // Solenoids
    //Air brake: 0, 3
    //Shifter: 2, 5
    //Intake: 1, 4
    public final static int DRIVE_SOL1 = 1; // needs to change //0 and 1 are the air brake
    public final static int DRIVE_SOL2 = 4; // needs to change

    // FEEDER CONSTANTS
    // Motor Controllers
    public final static int FEED = 6; // needs to change
    public final static int INDEX = 7; // needs to change

    // TURRET CONSTANTS
    // Motor Controllers
    public final static int TURR = 1; // needs to change

    // Position
    public final static double pos1 = -46.67 / 4;
    public final static double pos2 = 0;
    public final static double pos3 = 46.67 / 4;

    // PID
    public final static double kTurretP = 0.0005;
    public final static double kTurretI = 0.0;
    public final static double kTurretD = 0.0;
    public final static double kTurretFF = 0.0;
    public final static double kTurretIZ = 0.0;

    // INTAKE CONSTANTS
    // Motor Controllers
    public final static int INTAKE = 1; // needs to change

    // Solenoids
    public final static int INTAKE_SOL1 = 6;
    public final static int INTAKE_SOL2 = 7;

    // SENSOR CONSTANTS
    // Sensors
    public final static int HALL = 5;

    // JOYSTICK CONSTANTS
    // Joystick Ports
    public static final int zero = 0;
    public static final int one = 1;

    // Joystick Axes
    public static final int XL = 0;
    public static final int YL = 1;
    public static final int XR = 4;
    public static final int YR = 5;

    // other
    public static final int Y = 1;

    // CLIMBER CONSTANTS
    // MOTORS
    public final static int CLIMBER_MOT = 10000000; // needs to change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // SOLENOIDS
    public final static int CLIMBER_SOL = 10; // needs to change!!!!!!!!!!!!!!!!!!!!!!!!!!
    public final static int CLIMBER_SOL2 = 11; // needs to change!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // PID
    public static final double kclimberP = 0; // all need to change!!!
    public static final double kclimberI = 0; // all need to change!!!
    public static final double kclimberD = 0; // all need to change!!!
    public static final double kclimberFF = 0; // all need to change!!!
    public static final double kclimberIZ = 0; // all need to change!!!

    // OH THE HUMAnITY!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    // Characterization Toolsuite Constants
    public static final double ksVolts = 0.897, 
                               kvVoltSecondsPerMeter = 3.11, 
                               kaVoltSecondsSquaredPerMeter = 0.353,
                               kTrackwidthMeters = 0.629, 
                               kP = 12.8,
                               kD = 0.0, 
                               kMaxSpeedMetersPerSecond = 3.4, // Need to be changed
                               kMaxAccelerationMetersPerSecondSquared = 8.83, // Need to be changed
                               kRamseteB = 2, 
                               kRamseteZeta = 0.7;

    public static final DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // Encoder Constants
    public static final double kEncoderDistancePerPulse = (6 * Math.PI * 2.54 / 100) / (360);
     // ~= 0.001329

    public static final boolean kGyroReversed = false;

    //Path Planning Constants
    public static final double x = 2;
    public static final double y = -1;
}
