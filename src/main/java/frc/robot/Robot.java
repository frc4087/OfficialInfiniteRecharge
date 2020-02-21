/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.FileReader;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.DrivebaseAutoTest;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autoCommand;
  public SendableChooser<Command> autoChooser;
  public static RobotContainer m_robotContainer;
  public static Timer timer;
  double prevTime;
  
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_robotContainer.robotInit();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
    // SmartDashboard.putNumber("Left Encoder", m_robotContainer.m_driveAuto.left_encPos);
    // SmartDashboard.putNumber("Right Encoder", m_robotContainer.m_driveAuto.right_encPos);
    // SmartDashboard.putNumber("Left Vel", m_robotContainer.m_driveAuto.left_encVel);
    // SmartDashboard.putNumber("Right Vel", m_robotContainer.m_driveAuto.right_encVel);
    // SmartDashboard.putNumber("Gyro", m_robotContainer.m_driveAuto.getHeading());
    // SmartDashboard.putNumber("x", m_robotContainer.m_driveAuto.getPose().getTranslation().getX());
    // SmartDashboard.putNumber("y", m_robotContainer.m_driveAuto.getPose().getTranslation().getY());
    // SmartDashboard.putNumber("Heading", m_robotContainer.m_driveAuto.getPose().getRotation().getDegrees());

    // String trajectoryJSON = "output/BasicPath.wpilib.json"; 
    // Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    // try {
    //   Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
    // } catch (IOException e) {
    //   // TODO Auto-generated catch block
    //   e.printStackTrace();
    // }

    // double x = 
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  @Override
  public void autonomousInit() {
    m_robotContainer.m_driveAuto.resetOdometry(new Pose2d(new Translation2d(Constants.x, Constants.y), new Rotation2d()));
    // m_robotContainer.m_driveAuto.m_drive.feed();
    // m_robotContainer.m_turretBase.zeroTurret();
    // m_robotContainer.autoInit();
    m_autoCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autoCommand != null) {
      m_autoCommand.schedule();
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // m_robotContainer.m_driveAuto.m_drive.tankDrive(0.8, 0.8);
    //m_robotContainer.m_driveAuto.m_drive.feed();

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //m_robotContainer.m_driveAuto.resetEncoders();
    // m_robotContainer.m_turretBase.zeroTurret();
    //m_robotContainer.m_driveAuto.m_gyro.reset();
    if (m_autoCommand != null) {
      m_autoCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.teleop();

    //m_robotContainer.m_driveAuto.lfMotor.setVoltage(4);
    //m_robotContainer.m_driveAuto.rfMotor.setVoltage(-4);
    //m_robotContainer.m_driveAuto.m_drive.feed();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}