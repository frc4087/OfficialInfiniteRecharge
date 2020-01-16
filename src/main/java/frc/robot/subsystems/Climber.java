/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

    public final CANSparkMax teleclimb = new CANSparkMax(0,1); //PLEASEESES CHANGE THESE VALUES!
    public final CANSparkMax winchclimb = new CANSparkMax(0,1); //PLESESAESE CHANEG THESE VALUES!
//  public final CANSparkMax unamed_climber_motor_3 = new CANSparkMax(0,1); // please change these vlaues?
  
  public Climber() {
//WAIT FOR OTHER PARTS OF ROBOTS (STORAGE) BEFORE CONTIUING
  /*if(getthatcertainButton() = yup){
    dewit
  }*/
  }
}