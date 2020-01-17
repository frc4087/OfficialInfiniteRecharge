/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.LidarBase;
import frc.robot.subsystems.ShooterBase;



public class Aim extends CommandBase {
    private String gameData = DriverStation.getInstance().getGameSpecificMessage();
    private final ColorSensorBase m_CPMBase = new ColorSensorBase();
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
        while(detectedColor!=wantedColor){
            m_CPMBase.CPMotor.set(1);
        }
    }

    public void CPtoDist(){

    }

    
    @Override
    public void end(final boolean interrupted) {
    }

    
    @Override
    public boolean isFinished() {
        return false;
    }
}
