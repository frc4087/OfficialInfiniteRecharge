/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

public class DrivebaseAutoTest extends SubsystemBase {

  // public final ToggledSolenoid shifterPistons = new ToggledSolenoid(Constants.DRIVE_SOL1, Constants.DRIVE_SOL2);

  public final WPI_TalonSRX lfMotor = new WPI_TalonSRX(Constants.LF);
  public final WPI_TalonSRX lbMotor = new WPI_TalonSRX(Constants.LB);
  public final WPI_TalonSRX rfMotor = new WPI_TalonSRX(Constants.RF);
  public final WPI_TalonSRX rbMotor = new WPI_TalonSRX(Constants.RB);

  //We have to invert the right voltage and the left encoder
  public Encoder left_Enc = new Encoder(3, 2, true);
  public Encoder right_Enc = new Encoder(0, 1);
  
  public double left_encPos;
  public double right_encPos;

  public double left_encVel;
  public double right_encVel;

  //public final SpeedControllerGroup leftMotors = new SpeedControllerGroup(lfMotor, lbMotor);
  //public final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rfMotor, rbMotor);

  // public double left_encPos = -lfMotor.getSelectedSensorPosition() * Constants.kEncoderDistancePerPulse;
  // public double right_encPos = rfMotor.getSelectedSensorPosition() * Constants.kEncoderDistancePerPulse;

  // public double left_encVel = -lfMotor.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse;
  // public double right_encVel = rfMotor.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse;

  public DifferentialDrive m_drive = new DifferentialDrive(lfMotor, rfMotor);

  private final DifferentialDriveOdometry m_odometry;
  public AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public DrivebaseAutoTest() {
    lfMotor.setSafetyEnabled(false);
    rfMotor.setSafetyEnabled(false);
    lbMotor.setSafetyEnabled(false);
    rbMotor.setSafetyEnabled(false);

    //lfMotor.setInverted(true);
    //lbMotor.setInverted(true);

    left_Enc.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    right_Enc.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    //lfMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 0);
    //rfMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, 0);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
        new Pose2d(Constants.x, Constants.y, new Rotation2d()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // left_encPos = -(lfMotor.getSelectedSensorPosition()) * Constants.kEncoderDistancePerPulse;// *0.066497;
    // right_encPos = (rfMotor.getSelectedSensorPosition()) * Constants.kEncoderDistancePerPulse;// *0.066497;

    // left_encVel = -lfMotor.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse;
    // right_encVel = rfMotor.getSelectedSensorVelocity() * Constants.kEncoderDistancePerPulse;

    left_encPos = left_Enc.getDistance();
    right_encPos = right_Enc.getDistance();
  
    left_encVel = left_Enc.getRate();
    right_encVel = right_Enc.getRate();

    lbMotor.follow(lfMotor);
    rbMotor.follow(rfMotor);

    m_odometry.update(Rotation2d.fromDegrees(getHeading()), left_encPos, right_encPos);
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
    return new DifferentialDriveWheelSpeeds(left_encVel, right_encVel);
  }

  public void voltageControl(double leftVolts, double rightVolts) {
    lfMotor.setVoltage(leftVolts);
    rfMotor.setVoltage(-rightVolts);
    m_drive.feed();
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
    left_Enc.reset();
    right_Enc.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return ((left_encPos + right_encPos) / 2.0);
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
    return -m_gyro.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

}
