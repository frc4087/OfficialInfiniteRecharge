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
  private final LidarBase m_LidarBase = new LidarBase();
  private final ShooterBase m_ShooterBase = new ShooterBase();
  private final double dist = m_LidarBase.getDistance();
  private final double desired_A = m_ShooterBase.getwantedAngle(dist);
  private final double current_A = 360 * m_ShooterBase.angle_motor.getEncoder().getPosition();
  private final double error_A = current_A - desired_A;
  // private double previous_error_A;
  private final double desired_RPM = m_ShooterBase.getwantedRPM(dist);
  private final double current_RPM = m_ShooterBase.RPM_motor.getEncoder().getVelocity();
  private final double error_RPM = desired_RPM - current_RPM;

  // private double previous_error_RPM;
  /**
   * Creates a new Trajectory.
   */
  public Aim() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double steeringAdjust = Robot.m_robotContainer.m_visiontracking.pidX();
    Robot.m_robotContainer.m_drivebase.m_drive.arcadeDrive(0,
        steeringAdjust * Robot.m_robotContainer.m_visiontracking.getTarget());

    PID_Angle();

    PID_RPM();
  }

  public void PID_Angle() {
    final double kP = 0.2;// This is random, must be found
    final double kF = 10;// This is random, must be found
    final double value = error_A * kP + kF;
    m_ShooterBase.RPM_motor.set(value);
  }

  public void PID_RPM() {
    final double kP = 0.2;// This is random, must be found
    final double value = error_RPM * kP;
    m_ShooterBase.RPM_motor.set(value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
