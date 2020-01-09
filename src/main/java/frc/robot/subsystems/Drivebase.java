/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {
  /**
   * Creates a new Drivebase.
   */
  private static final MotorType kMotorType = MotorType.kBrushless;

  Constants m_constants = new Constants();

  public Drivebase() {

  }

    private final CANSparkMax left_ff = new CANSparkMax(m_constants.LFF, kMotorType);
    private final CANSparkMax left_mf = new CANSparkMax(m_constants.LMF, kMotorType);
    private final CANSparkMax left_bb = new CANSparkMax(m_constants.LBB, kMotorType);
    private final CANSparkMax left_mb = new CANSparkMax(m_constants.LMB, kMotorType);
    private final CANSparkMax right_mf = new CANSparkMax(m_constants.RMF, kMotorType);
    private final CANSparkMax right_ff = new CANSparkMax(m_constants.RFF, kMotorType);
    private final CANSparkMax right_bb = new CANSparkMax(m_constants.RBB, kMotorType);
    private final CANSparkMax right_mb = new CANSparkMax(m_constants.RMB, kMotorType);
    private final Encoder l_encoder = new Encoder(0, 1);//params are not real
    private final Encoder r_encoder = new Encoder(2, 3);//params are not real
    private final SpeedControllerGroup righkkkt_side = new SpeedControllerGroup(right_mf, right_ff, right_mb, right_bb);
    private final SpeedControllerGroup lefkkkt_side = new SpeedControllerGroup(left_mf, left_ff, left_mb, left_bb);
    public final DifferentialDrive m_drive = new DifferentialDrive(lefkkkt_side, righkkkt_side);
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
