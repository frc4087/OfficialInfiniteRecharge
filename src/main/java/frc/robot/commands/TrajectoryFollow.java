/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivebaseAuto;

//TO DO
//MAKE A PID CONTROLLER SUBSYSTEM AND COMMAND 

public class TrajectoryFollow extends CommandBase {

  DrivebaseAuto m_drivebaseAuto = new DrivebaseAuto();

  /**
   * Creates a new TrajectoryFollow.
   */
  public TrajectoryFollow() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(DrivebaseAuto);
  }

  public Command getAutonomousCommand() throws IOException {

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
        Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.m_driveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.m_driveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    /*
     * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory( //
     * Start at the origin facing the +X direction new Pose2d(0, 0, new
     * Rotation2d(0)), // Pass through these two interior waypoints, making an 's'
     * curve path List.of( new Translation2d(1, 1), new Translation2d(2, -1) ), //
     * End 3 meters straight ahead of where we started, facing forward new Pose2d(3,
     * 0, new Rotation2d(0)), // Pass config config );
     */

    Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/BluePos1Search.wpilib.json"));
    //Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/BluePos1Search.wpilib.json"));
  
    
    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drivebaseAuto::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.m_driveKinematics, 
        m_drivebaseAuto::getWheelSpeeds, 
        //m_drivebaseAuto::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0), new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivebaseAuto::voltageControl,
        m_drivebaseAuto);


    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> Robot.m_robotContainer.m_drivebaseAuto.voltageControl(0, 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
