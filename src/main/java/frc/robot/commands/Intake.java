/*package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;


public class Intake extends CommandBase {

    @Override
    public void execute() {  
      //This will turn on the intake to collect power cells
     if(driveJoy.getYButton()){
       Robot.m_IntakeBase.IntakeIn.set(-0.5);
      }else{
        Robot.m_IntakeBase.IntakeIn.set(0);
     }
     //this will turn on the outake to expell power cells
     if(driveJoy.getBButton()){
        Robot.m_IntakeBase.IntakeOut.set(-0.5);
       }else{
        Robot.m_IntakeBase.IntakeOut.set(0);
      }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}*/