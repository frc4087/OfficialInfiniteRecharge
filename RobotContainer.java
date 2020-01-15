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
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.LidarBase;
import frc.robot.subsystems.PathPlannerBase;
//import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterBase;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivebase m_drivebase = new Drivebase();
  public static final Constants m_constants = new Constants();
  public final PathPlannerBase m_pathPlannerBase = new PathPlannerBase();
  public final LidarBase m_lidarBase = new LidarBase();
  public final ShooterBase m_shooterBase = new ShooterBase();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

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
    if(getDriveJoy(Constants.YL) > 0.5){
    m_drivebase.m_drive.curvatureDrive(getDriveJoy(Constants.YL), getDriveJoy(Constants.XR), isQuickTurn());
  } else {
    m_drivebase.m_drive.arcadeDrive(getDriveJoy(Constants.YL), getDriveJoy(Constants.XR));
  }
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
    return m_autoCommand;
  }*/
}
