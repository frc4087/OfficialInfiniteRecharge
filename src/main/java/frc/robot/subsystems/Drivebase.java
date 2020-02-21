/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivebase extends SubsystemBase {

  public final ToggledSolenoid shifterPistons = new ToggledSolenoid(Constants.DRIVE_SOL1, Constants.DRIVE_SOL2);

  public final CANSparkMax lfMotor = new CANSparkMax(Constants.LF, MotorType.kBrushless);
  public final CANSparkMax lbMotor = new CANSparkMax(Constants.LB, MotorType.kBrushless);
  public final CANSparkMax rfMotor = new CANSparkMax(Constants.RF, MotorType.kBrushless);
  public final CANSparkMax rbMotor = new CANSparkMax(Constants.RB, MotorType.kBrushless);

  public final SpeedControllerGroup leftMotors = new SpeedControllerGroup(lfMotor, lbMotor);
  public final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rfMotor, rbMotor);

  public DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  public Drivebase() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
