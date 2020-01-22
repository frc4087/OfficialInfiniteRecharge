/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class IntakeOut extends CommandBase {
  public IntakeOut() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_robotContainer.m_IntakeBase);
  }


 // Called when the command is initially scheduled.
 @Override
 public void initialize() {
 }



  // Called every time the scheduler runs while the command is scheduled.
 @Override
 public void execute() {  
  Robot.m_robotContainer.m_intakeBase.RIntake.set(0.5);
  Robot.m_robotContainer.m_intakeBase.LIntake.set(-0.5);
}

  // Called once the command ends or is interrupted.
 @Override
 public void end(boolean interrupted) {
 }
  // Returns true when the command should end.
 @Override
 public boolean isFinished() {
   return false;
 }
}
