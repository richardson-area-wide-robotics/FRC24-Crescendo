// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.JoystickUtil;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter m_shooter = new Shooter();
  private final AHRS m_gyro = new AHRS();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro);

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(IOConstants.kDriverControllerPort);
  XboxController m_driverControllerSP = new XboxController(IOConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(IOConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the trigger bindings
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
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
    // set up for driving controls
    DoubleSupplier moveForward = () -> MathUtil.applyDeadband(
        -m_driverController.getLeftY(), Constants.IOConstants.kControllerDeadband);
    DoubleSupplier moveSideways = () -> MathUtil.applyDeadband(
        -m_driverController.getLeftX(), Constants.IOConstants.kControllerDeadband);

    // Configure default commands
    /**
     * ---Driving Controls for the driver
     * The left stick on Xbox controller controls the translation of the robot - 1
     * The right stick controls the rotation of the robot - 12
     */
    m_robotDrive.setDefaultCommand(
        Commands.run(
            () -> m_robotDrive.drive(
                moveForward.getAsDouble(),
                moveSideways.getAsDouble(),
                JoystickUtil.squareAxis(
                    -m_driverController.getRightX()),
                true),
            m_robotDrive));

    /*
     * ---Reset button and X mode button
     * left stick button on controller controls the re-zeroing of the heading
     * right stick button on controller controls the X mode of the robot
     */

    m_driverController.rightStick().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.leftStick().onTrue(Commands.runOnce(() -> m_robotDrive.setX()));

    /**
     * SHOOTER
     */
    m_shooter.setDefaultCommand(Commands.run(() -> {
      m_shooter.idle();
    }, m_shooter));

    m_driverController
        .a()
        .onTrue(Commands.run(() -> {
          m_shooter.speakerFire();
        }, m_shooter));

    m_driverController
        .b()
        .whileTrue(Commands.run(() -> {
          m_shooter.speakerMode();
        }, m_shooter));

    // Temporary pivot button bindings for testing
    m_driverController
        .leftBumper()
        .onTrue(Commands.run(() -> {
          m_shooter.pivotTo(Degrees.of(10.0));
        }, m_shooter));

    m_driverController
        .rightBumper()
        .onTrue(Commands.run(() -> {
          m_shooter.pivotTo(Degrees.of(45.0));
        }, m_shooter));
  }

  private void configureOperatorBindings() {
    /*
     * All of the operator controls will go here
     */

  }

  /** Run a function at the start of auton. */
  public void autonInit() {
    // m_robotDrive.calibrateGyro();
    // m_robotDrive.stop();
    this.globalEventList();
  }

  /** Creates the Global event list for the autonomous paths */
  public void globalEventList() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // System.out.println("Auton Selected" + AutoChooser.getAuton().getName());
    // return AutoChooser.getAuton();
    PathPlannerPath path = PathPlannerPath.fromPathFile("example");
    return AutoBuilder.followPath(path);
    // return new PathPlannerAuto("test");
  }

  /**
   * Use this method to pass anythign to the dashboard
   * 
   * Reduces multi method use to Shuffleboard
   */
  public void putDashboard() {
    // m_robotDrive.putNumber();
    // SmartDashboard.putNumber("filtered PoseX", m_robotDrive.getPose().getX());
    // SmartDashboard.putNumber("filtered PoseY", m_robotDrive.getPose().getY());
  }

  /**
   * Sets the idle mode for the arm and intake joints
   * 
   * Used to make the robot arm easier to move when disabled
   */
  public void setIdleMode(IdleMode idleMode) {

  }

  /** Run a function during autonomous to get run time of autonomous. */
  public void autonPeriodic() {
    SmartDashboard.putNumber("Auton Time", Timer.getFPGATimestamp());

  }
}