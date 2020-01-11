/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ObjectAvoidance extends CommandBase {
  int stopD = 5;
  int width = 60;
  int halfWidth = width/2;
  int extention = width + 15;
  double desiredA;
  double error; 
  double previous_error;
  double currentA = Robot.m_robotContainer.m_drivebase.gyro.getAngle();

  
  public ObjectAvoidance() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.m_robotContainer.m_pathPlannerBase, Robot.m_robotContainer.m_lidarBase, Robot.m_robotContainer.m_drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  if(Robot.m_robotContainer.m_lidarBase.getDistance() < stopD){
      Robot.m_robotContainer.m_drivebase.m_drive.arcadeDrive(0, 0);
  } else {
    Robot.m_robotContainer.m_pathPlannerBase.followPath();
  }
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
