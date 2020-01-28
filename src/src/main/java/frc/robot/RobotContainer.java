/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CPMBase;
import frc.robot.subsystems.ColorMatcher;
// import frc.robot.subsystems.Drivebase;
// import frc.robot.subsystems.LidarBase;
// import frc.robot.subsystems.LidarBaseCopy;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // final Drivebase m_drivebase = new Drivebase();
  // final LidarBaseCopy m_lidarBaseCopy = new LidarBaseCopy(Port.kOnboard);
  // final LidarBase m_lidarBase = new LidarBase();
  final CPMBase m_CPMBase = new CPMBase();
  final ColorMatcher m_colorMatcher = new ColorMatcher();

  //Control Scheme
  public static final double JOY_DEADZONE = 0.1;
  boolean quickTurn = false;
    // Initialize joysticks
    public final XboxController driveJoy = new XboxController(0);
    public final XboxController opJoy = new XboxController(1);

    //Joystick Methods
    public double getDriveJoy(int axis){
      double raw = driveJoy.getRawAxis(axis);
      return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
    }

    public double getOpJoy(int axis){
      double raw = opJoy.getRawAxis(axis);
      return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
    }
    
    public boolean isQuickTurn() {
      if (getDriveJoyBRPressed()) {
        quickTurn = !quickTurn;
      }
      return quickTurn;
    }

    public boolean getDriveJoyBRPressed() {
      return driveJoy.getBumperPressed(Hand.kRight);
    }

    public double getDriveJoyXR() {
      double raw = driveJoy.getRawAxis(4);
      if (isQuickTurn()) {
        return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw > 0 ? (raw * raw) / 2 : (-raw * raw) / 2;
      } else {
        return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
      }
    }

  //Match Period Methods
  public void teleop(){
    /*if (driveJoy.getXButton()) {
      new Aim();
    } else {
      if(getDriveJoy(Constants.YL) > 0.5){
        m_drivebase.m_drive.curvatureDrive(getDriveJoy(Constants.YL), getDriveJoy(Constants.XR), isQuickTurn());
      } else {
        m_drivebase.m_drive.arcadeDrive(getDriveJoy(Constants.YL), getDriveJoy(Constants.XR));
      }
    }*/

    
    
    // if (driveJoy.getAButton()) {
    //   if(SmartDashboard.getNumber("Lidar Copy", 0)>40.0){
        
    //     m_drivebase.m_drive.arcadeDrive(0.5, 0.0);
    //   }
     
    // } else{
      if(driveJoy.getAButton()){
        // if(SmartDashboard.getString("Detected Color", "")!="Blue"){
          m_CPMBase.CPMotor.set(0.9);
        // }else{
        //   m_CPMBase.CPMotor.set(0.0);
        // }
      // }else{
      //   if(Math.abs(getDriveJoy(Constants.YL)) > 0.3){
      //     m_drivebase.m_drive.curvatureDrive(-getDriveJoy(Constants.YL), getDriveJoy(Constants.XR), isQuickTurn());
      //   } else {
      //     m_drivebase.m_drive.arcadeDrive(-getDriveJoy(Constants.YL), getDriveJoy(Constants.XR));
      //   }
      }else{
        m_CPMBase.CPMotor.set(0.0);
      }
    // }
    

   }


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return m_autoCommand;
  }*/
}
