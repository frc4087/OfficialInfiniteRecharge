/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {

  public Drivebase() {
  }

    public final CANSparkMax left_ff = new CANSparkMax(Constants.LFF, Constants.kMotorType);
    public final CANSparkMax left_mf = new CANSparkMax(Constants.LMF, Constants.kMotorType);
    public final CANSparkMax left_bb = new CANSparkMax(Constants.LBB, Constants.kMotorType);
    public final CANSparkMax left_mb = new CANSparkMax(Constants.LMB, Constants.kMotorType);
    public final CANSparkMax right_ff = new CANSparkMax(Constants.RFF, Constants.kMotorType);
    public final CANSparkMax right_mf = new CANSparkMax(Constants.RMF, Constants.kMotorType);
    public final CANSparkMax right_bb = new CANSparkMax(Constants.RBB, Constants.kMotorType);
    public final CANSparkMax right_mb = new CANSparkMax(Constants.RMB, Constants.kMotorType);
    
    public final SpeedControllerGroup right_side = new SpeedControllerGroup(right_mf, right_ff, right_mb, right_bb);
    public final SpeedControllerGroup left_side = new SpeedControllerGroup(left_mf, left_ff, left_mb, left_bb);
    public final DifferentialDrive m_drive = new DifferentialDrive(left_side, right_side);

    public final Encoder l_encoder = new Encoder(0, 1);//params are not real
    public final Encoder r_encoder = new Encoder(2, 3);//params are not real

    public AHRS gyro = new AHRS(SPI.Port.kMXP);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    left_mf.follow(left_ff);
    left_bb.follow(left_ff);
    left_mb.follow(left_ff);

    right_mf.follow(right_ff);
    right_bb.follow(right_ff);
    right_mb.follow(right_ff);
  }
}