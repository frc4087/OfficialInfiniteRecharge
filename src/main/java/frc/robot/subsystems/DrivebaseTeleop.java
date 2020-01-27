/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivebaseTeleop extends SubsystemBase {
  /**
   * Creates a new Drivebase.
   */
  public DrivebaseTeleop() {
  }

  public final WPI_TalonSRX left_f = new WPI_TalonSRX(Constants.LF);
  public final WPI_TalonSRX left_b = new WPI_TalonSRX(Constants.LB);
  public final WPI_TalonSRX right_f = new WPI_TalonSRX(Constants.RF);
  public final WPI_TalonSRX right_b = new WPI_TalonSRX(Constants.RB);

  public final SpeedControllerGroup left_side = new SpeedControllerGroup(left_f, left_b);
  public final SpeedControllerGroup right_side = new SpeedControllerGroup(right_f, right_b);
  public final DifferentialDrive m_drive = new DifferentialDrive(left_side, right_side);

  private final Gyro m_gyro = new ADXRS450_Gyro();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}