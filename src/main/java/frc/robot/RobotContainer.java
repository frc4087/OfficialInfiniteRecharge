/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

// import java.util.Map;

// import com.revrobotics.ControlType;

// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.I2C.Port;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveBackwards;
import frc.robot.commands.DriveForwards;
// import frc.robot.subsystems.CPMBase;
// import frc.robot.subsystems.ClimberBase;
//import edu.wpi.first.wpilibj2.command.button.POVButton;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.DrivebaseAutoTest;
// import frc.robot.subsystems.LidarBase;
// import frc.robot.subsystems.LimelightBase;
// import frc.robot.subsystems.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 // private final Drivebase m_drivebase = new Drivebase();
  // private final FeederBase m_feederBase = new FeederBase();
  //private final IntakeBase m_intakeBase = new IntakeBase();
  // public final TurretBase m_turretBase = new TurretBase();
  // public final LimelightBase m_limelightBase = new LimelightBase();
  // public final ClimberBase m_climberBase = new ClimberBase();
  // public final LidarBase m_lidarBase = new LidarBase(Port.kOnboard);
  // public final CPMBase m_CPMBase = new CPMBase();
 public final DrivebaseAutoTest m_driveAuto = new DrivebaseAutoTest();
 //public final Command m_driveBackwards = new DriveBackwards();
  //public final Command m_driveForwards = new DriveForwards();
  public Trajectory trajectory;
  //public SendableChooser<Command> autoChooser;
  //private Counter m_LIDAR;

  //Control Scheme
  public static final double JOY_DEADZONE = 0.1;
  boolean quickTurn = false;
  boolean shiftState = false; //check this later
  
    //Buttons
    //public final boolean upPOVButton = new POVButton().getUp();
    // public final boolean rightPOVButton = new POVButton().getRight();
    // public final boolean downPOVButton = new POVButton().getDown();
    // public final boolean leftPOVButton = new POVButton().getLeft();

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
    
    // public boolean isQuickTurn() {
    //   if (getDriveJoyBRPressed()) {
    //     quickTurn = !quickTurn;
    //   }
    //   return quickTurn;
    // }

    // public boolean getDriveJoyBRPressed() {
    //   return driveJoy.getBumperPressed(Hand.kRight);
    // }

    // public double getDriveJoyXR() {
    //   double raw = driveJoy.getRawAxis(4);
    //   if (isQuickTurn()) {
    //     return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw > 0 ? (raw * raw) / 2 : (-raw * raw) / 2;
    //   } else {
    //     return Math.abs(raw) < JOY_DEADZONE ? 0.0 : raw;
    //   }
    // }

    //Match Period Code
    public void robotInit(){
      //SmartDashboard.putNumber("Auto Chooser", 0);
      // m_LIDAR = new Counter(0); //plug the lidar into PWM 0
      // m_LIDAR.setMaxPeriod(1.00); //set the max period that can be measured
      // m_LIDAR.setSemiPeriodMode(true); //Set the counter to period measurement
      // m_LIDAR.reset();

      // autoChooser = new SendableChooser<Command>();
      // autoChooser.setDefaultOption("Drive Backwards", m_driveBackwards);
      // autoChooser.addOption("Drive Backwards", m_driveBackwards);
      // autoChooser.addOption("Drive Forwards", m_driveForwards);//.addDefault("Drive Backwards", m_driveBackwards);
      // SmartDashboard.putData("Auto Mode", autoChooser);
     }
    public void autoInit(){
      // SmartDashboard.putNumber("Auto Chooser", 0);
      // while(SmartDashboard.getNumber("Auto Chooser", 0)==0){
      //   if(SmartDashboard.getNumber("Auto Chooser", 0)!=0){
      //     break;
      //   }
      // }
    }
    public void teleop(){
     //DRIVE JOYSTICK
      //DRIVEBASE
      //m_drivebase.m_drive.curvatureDrive(-Math.pow(getDriveJoy(Constants.YL), 3), getDriveJoy(Constants.XR), driveJoy.getBButtonPressed());
      //SmartDashboard.putNumber("Gyro", m_drivebase.m_gyro.getYaw());
    
      // if (driveJoy.getXButtonPressed()) {
      //   m_drivebase.shifterPistons.togglePiston();
      //   shiftState = true;
      // }
      
    
      //  SmartDashboard.putString("Gear", shiftState ? "Low" : "High");

    //  //OPERATOR JOYSTICK
    //   //INTAKE
      // if(driveJoy.getTriggerAxis(Hand.kLeft) > 0.05){
      //  m_intakeBase.intakeMotor.set(driveJoy.getTriggerAxis(Hand.kLeft));
      // } else {
      //  m_intakeBase.intakeMotor.set(-driveJoy.getTriggerAxis(Hand.kRight));
      // }
      //  SmartDashboard.putNumber("left trigger", driveJoy.getTriggerAxis(Hand.kLeft));
      //  SmartDashboard.putNumber("right trigger", driveJoy.getTriggerAxis(Hand.kRight));

    
    //   if (opJoy.getXButtonPressed()) {
    //     m_intakeBase.intakePistons.togglePiston();
    //   }

    //   if(opJoy.getBumperPressed(Hand.kRight)){
    //     m_CPMBase.CPMtask.execute();
    //   }

    //   //FEEDER AND INDEXER
    //   m_feederBase.feederMotor.set(getOpJoy(Constants.YR));
       //m_feederBase.indexMotor.set(getDriveJoy(Constants.YL));
    //   m_turretBase.checkTurret();
    }
    //   //LAUNCHER
    //   m_turretBase.turretMotor.set(getOpJoy(Constants.XR));
    //   SmartDashboard.putNumber("Turret Encoder", m_turretBase.turretMotor.getEncoder().getPosition());
    //   SmartDashboard.putBoolean("Hall Effect", m_turretBase.hallEffect.get());

    //   if (leftPOVButton) {
    //     m_turretBase.turretPID.setReference(Constants.pos1, ControlType.kPosition);
    //   }

    //   if (upPOVButton) {
    //     m_turretBase.turretPID.setReference(Constants.pos2, ControlType.kPosition);
    //   }

    //   if (rightPOVButton) {
    //     m_turretBase.turretPID.setReference(Constants.pos3, ControlType.kPosition);
    //   }

    //   //LIMELIGHT 
    //   if (opJoy.getAButton()) {
    //     m_turretBase.turretPID.setReference(m_limelightBase.pidX()* m_limelightBase.get("tv"), ControlType.kPosition);
    //   }

    //   //LIDAR
    //   SmartDashboard.putNumber("Lidar", m_lidarBase.getDistance());

    //   //CLIMBER
    //   m_climberBase.braker();
 
    //   if(opJoy.getYButton()){
    //   m_climberBase.climberPID.setReference(100, ControlType.kVelocity);
    //    }else{
    //   m_climberBase.climberPID.setReference(0, ControlType.kVelocity);
    //    }
    // }
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
   public Command getAutonomousCommand() {
    //double val = SmartDashboard.getNumber("Auto Chooser", 0);
    //return val == 1 ? m_driveForwards : val == 2 ? m_driveBackwards : null;
    return pathFollow();
  }


  public Command pathFollow() {

    // Create a voltage constraint to ensure we don't accelerate too fast
    final var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.m_driveKinematics, 10);

    // // Create config for trajectory
    final TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
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

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(2, -1, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(2.5, -1.25),
          new Translation2d(3, -1.5),
          new Translation2d(3.5, -1.75),
          new Translation2d(4, -2),
          new Translation2d(3.5, -2.25),
          new Translation2d(3, -2.5),
          new Translation2d(2.5, -2.75)
      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(2, -3, new Rotation2d(180)),
      // Pass config
      config
  );

    String trajectoryJSON = "output/BasicPath.wpilib.json"; 
    try {
      Path testTrajectory = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(testTrajectory);
      // trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON));
     } catch (final IOException ex) {
       // TODO Auto-generated catch block
       DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
     }
    final RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory, 
        m_driveAuto::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.m_driveKinematics, 
        m_driveAuto::getWheelSpeeds,
        new PIDController(Constants.kP, 0, 0), 
        new PIDController(Constants.kP, 0, 0),
        m_driveAuto::voltageControl,
        m_driveAuto);
    // Run path following command, then stop at the end.
    //Robot.m_robotContainer.m_driveAuto.m_drive.feed();
    return ramseteCommand.andThen(() -> Robot.m_robotContainer.m_driveAuto.voltageControl(0, 0));
  }
}
