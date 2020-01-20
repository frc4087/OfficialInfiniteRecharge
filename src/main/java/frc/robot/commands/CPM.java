/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.CPMBase;




public class CPM extends CommandBase {
    private String gameData = DriverStation.getInstance().getGameSpecificMessage();
    private final CPMBase m_CPMBase = new CPMBase();
    private Color detectedColor = m_CPMBase.m_colorSensor.getColor();
    private Color wantedColor;
    
    
    public CPM() {
        
    }

    
    @Override
    public void initialize() {
    }

    
    @Override
    public void execute() {
        if(gameData.length()>0){
            wantedColor = Constants.FMStoColor.get(gameData);
            CPtoColor();
        }else{
            CPtoDist();
        }
    }

    public void CPtoColor(){
        if(detectedColor!=wantedColor){
            m_CPMBase.CPMotor.set(1);
        }else{
            m_CPMBase.CPMotor.set(0);
        }
    }

    public void CPtoDist(){
      int count = 0;
      Color start = detectedColor;
      boolean alreadyCounted = true;
      m_CPMBase.CPMotor.set(1);
      while(count<8){
        if(detectedColor==start){
          if(!alreadyCounted){
            count++;
            alreadyCounted = true;
          }
        }else{
          alreadyCounted = false;
        }
      }
      m_CPMBase.CPMotor.set(0);
    }

    public void CPtoTime(){
      
    }
    
    @Override
    public void end(final boolean interrupted) {
    }

    
    @Override
    public boolean isFinished() {
        return false;
    }
}
