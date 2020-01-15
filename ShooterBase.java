/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Add your docs here.
 */
public class ShooterBase extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public CANSparkMax angle_motor = new CANSparkMax(Constants.kShooterAngleMotor, Constants.kMotorType);
  public CANSparkMax RPM_motor = new CANSparkMax(Constants.kShooterRPMMotor, Constants.kMotorType);
   //LIMIT SWITCHES:
   DigitalInput limitSwitch = new DigitalInput(1);
  
  public ShooterBase() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  } 

  if(limitSwitch.get() == false) { 
    angle_motor.getEncoder().setPosition(0.0);
    //RESETS THE ENCODER POSITION FOR SHOOTER
  }

  public double getwantedRPM(double dist){
    return 15.402*Math.pow(dist, 2) - 27.191 * dist + 5471.8;
  }

  public double getwantedAngle(double dist){
    return 0.1106*Math.pow(dist, 2) - 5.053 * dist + 87.522;
  }
}
