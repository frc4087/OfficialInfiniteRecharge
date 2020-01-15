/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.io.IOException;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.followers.EncoderFollower;

public class PathPlannerBase extends SubsystemBase {
  
  private static final int k_ticks_per_rev = 8192;
  private static final double k_wheel_diameter = 6;
  private static final double k_max_velocity = 14;

  private EncoderFollower m_left_follower;
  private EncoderFollower m_right_follower;
  Notifier m_follower_notifier;

  public void pathSelector(String pathName){
    try {
      jaci.pathfinder.Trajectory left_trajectory = PathfinderFRC.getTrajectory(pathName + ".left");
      jaci.pathfinder.Trajectory right_trajectory = PathfinderFRC.getTrajectory(pathName + ".right");
  
      m_left_follower = new EncoderFollower(left_trajectory);
      m_right_follower = new EncoderFollower(right_trajectory);
  
      m_left_follower.configureEncoder((int)Robot.m_robotContainer.m_drivebase.left_ff.getEncoder().getPosition()*8172, k_ticks_per_rev, k_wheel_diameter);
      // You must tune the PID values on the following line!
      m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
  
      m_right_follower.configureEncoder((int)Robot.m_robotContainer.m_drivebase.right_ff.getEncoder().getPosition(), k_ticks_per_rev, k_wheel_diameter);
      // You must tune the PID values on the following line!
      m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
  
      m_follower_notifier = new Notifier(this::followPath);
      m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void followPath() {
    if (m_left_follower.isFinished() || m_right_follower.isFinished()) {
      m_follower_notifier.stop();
    } else {
      double left_speed = m_left_follower.calculate(Robot.m_robotContainer.m_drivebase.l_encoder.get());
      double right_speed = m_right_follower.calculate(Robot.m_robotContainer.m_drivebase.r_encoder.get());
      double heading = Robot.m_robotContainer.m_drivebase.gyro.getAngle();
      double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn =  0.8 * (-1.0/80.0) * heading_difference;
      Robot.m_robotContainer.m_drivebase.left_side.set(left_speed + turn);
      Robot.m_robotContainer.m_drivebase.right_side.set(right_speed - turn);
    }
  }
  public PathPlannerBase() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
