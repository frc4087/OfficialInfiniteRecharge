/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

//TO MODIFY:
//FIGURE OUT HOW TO SET DISTANCE PER PULSE OF ENCODERS
//TRY TO SET MOTOR VOLTAGE USING setVoltage() rather than set()

public class DrivebaseAuto extends SubsystemBase {
  /**
   * Creates a new Drivebase.
   */
  public DrivebaseAuto() {
    //configEncoderCodesPerRev(90);

    //left_f.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    //right_f.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public final TalonSRX left_f = new TalonSRX(Constants.LF);
  public final TalonSRX left_b = new TalonSRX(Constants.LB);
  public final TalonSRX right_f = new TalonSRX(Constants.RF);
  public final TalonSRX right_b = new TalonSRX(Constants.RB);

  public double left_encPos = left_f.getSelectedSensorPosition();
  public double right_encPos = right_f.getSelectedSensorPosition();

  

  private final Gyro m_gyro = new ADXRS450_Gyro();

  private final DifferentialDriveOdometry m_odometry;

  //Encoder m_leftEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k2X);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), left_f.getSelectedSensorPosition(), right_f.getSelectedSensorPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left_f.getSelectedSensorVelocity(), right_f.getSelectedSensorVelocity());
  }

  public void voltageControl(ControlMode mode, double leftVolts, double rightVolts) {
    left_f.set(mode, leftVolts);
    right_f.set(mode, -rightVolts);

 	}

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(final Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    left_f.setSelectedSensorPosition(0);
    right_f.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((left_f.getSelectedSensorPosition() + right_f.getSelectedSensorPosition()) / 2.0);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }
}

