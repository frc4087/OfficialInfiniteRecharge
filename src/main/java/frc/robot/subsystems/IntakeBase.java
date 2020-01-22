/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class IntakeBase extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  
  public IntakeBase() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public final CANSparkMax RIntake =  new CANSparkMax(Constants.kRIntake, Constants.kMotorType);
  final public CANSparkMax LIntake = new CANSparkMax(Constants.kLIntake, Constants.kMotorType);
  final public SpeedControllerGroup IntakeMotors = new SpeedControllerGroup(RIntake, LIntake);
}
