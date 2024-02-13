// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.JoystickUtil;
import frc.robot.Constants.IOConstants;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Camera m_camera = new Camera("camera");

  // The robot's subsystems
  // private final AHRS m_gyro = new AHRS();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private Boolean intakeMode = false;
  private double speed = -0.1;
  private double intakeSpeed = 1.0;
  private double outtakingSpeed = -1.0;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(IOConstants.kDriverControllerPort);
  XboxController m_driverControllerSP = new XboxController(IOConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(IOConstants.kOperatorControllerPort);

  {
    // AutoChooser.setDefaultAuton(new PathTester(m_robotDrive));
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
   
    // Configure the trigger bindings
    configureDriverBindings(); 
    configureOperatorBindings();   
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureDriverBindings() {

    // TODO: remove this before merging
    // new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(balance);

    // set up for driving controls
    DoubleSupplier moveForward =  () -> MathUtil.applyDeadband(
      -m_driverController.getLeftY(), Constants.IOConstants.kControllerDeadband);
     DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
      -m_driverController.getLeftX(), Constants.IOConstants.kControllerDeadband);
    
    // Configure default commands
    /* 
       * ---Driving Controls for the driver 
       * The left stick on Xbox controller controls the translation of the robot - 1
       * The right stick controls the rotation of the robot - 12
       */
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () ->
          m_robotDrive.drive(
          moveForward.getAsDouble(),
          moveSideways.getAsDouble(),
          JoystickUtil.squareAxis(
          -m_driverController.getRightX()),
          true, true), m_robotDrive));

      /*
       * ---Reset button and X mode button
       * left stick button on controller controls the re-zeroing of the heading 
       * right stick button on controller controls the X mode of the robot
       */

       m_driverController.rightStick().onTrue(new InstantCommand(()-> m_robotDrive.zeroHeading()));
       m_driverController.leftStick().onTrue(new InstantCommand(()-> m_robotDrive.setX()));
  }

  private void configureOperatorBindings(){
   /*
    * All of the operator controls will go here 
    */

  }

   /** Run a function at the start of auton. */
   public void autonInit(){
    // m_robotDrive.calibrateGyro();
    // m_robotDrive.stop();
    this.globalEventList();
  }

  /** Creates the Global event list for the autonomous paths */
  public void globalEventList(){}
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

  /**
   * Use this method to pass anythign to the dashboard 
   * 
   * Reduces multi method use to Shuffleboard
   */
  public void putDashboard(){
    // m_robotDrive.putNumber();
    // SmartDashboard.putNumber("filtered PoseX", m_robotDrive.getPose().getX());
    // SmartDashboard.putNumber("filtered PoseY", m_robotDrive.getPose().getY());
  }

    /**
   * Sets the idle mode for the arm and intake joints
   * 
   * Used to make the robot arm easier to move when disabled
   */
  public void setIdleMode(IdleMode idleMode){

  }

/** Run a function during autonomous to get run time of autonomous. */
public void autonPeriodic(){
  SmartDashboard.putNumber("Auton Time", Timer.getFPGATimestamp());

}
}