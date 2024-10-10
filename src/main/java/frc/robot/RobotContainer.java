// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.util.JoystickUtil;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LockMode;
import frc.robot.Constants.PivotConstants.PivotDirection;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.auton.BackUp;
import frc.robot.auton.ShootBackUp;
import frc.robot.auton.TwoShootBasicAuto;
import frc.robot.auton.ThreeShootLeftAuto;
import frc.robot.auton.ThreeShootRightAuto;
import frc.robot.auton.FrontFourLeftToRightAuto;
import frc.robot.auton.FrontFourRightToLeftAuto;
import frc.robot.auton.ShootNoBackUp;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import frc.robot.commands.Lock;
import frc.robot.subsystems.Camera;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants.ClimberDirection;
import frc.robot.subsystems.climber.Climber;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake m_intake = new Intake();
  private final Pivot m_pivot = new Pivot();
  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder();
  private final AHRS m_gyro = new AHRS();
  private final Camera m_camera = new Camera("camera");
  public final DriveSubsystem m_robotDrive = new DriveSubsystem(m_gyro, m_camera);
  private final Climber m_climber = new Climber();
  private final ShootBackUp m_shootBackUp = new ShootBackUp(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final TwoShootBasicAuto m_twoShootBasicAuto = new TwoShootBasicAuto(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final ThreeShootLeftAuto m_threeShootLeftAuto = new ThreeShootLeftAuto(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final ThreeShootRightAuto m_threeShootRightAuto = new ThreeShootRightAuto(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final FrontFourLeftToRightAuto m_frontFourLeftToRightAuto = new FrontFourLeftToRightAuto(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final FrontFourRightToLeftAuto m_frontFourRightToLeftAuto = new FrontFourRightToLeftAuto(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final BackUp m_backUp = new BackUp(m_robotDrive);
  private final ShootNoBackUp m_shootNoBackUp = new ShootNoBackUp(m_robotDrive, m_intake, m_shooter, m_pivot, m_feeder);
  private final Command commandIntake = m_intake.intake();
  private final Command commandOuttake = m_intake.outtake();
  private final Command commandPivotToSpeaker = m_pivot.pivotToSpeaker();
  private final Command commandShoot = Commands.run(() -> {
          m_shooter.toggleState(ShooterState.SPEAKER);
        }, m_shooter).andThen(Commands.runOnce(() -> {
          m_feeder.shootNote();
        }));
  private final Command commandIdle = Commands.run(() -> m_shooter.idle());
  private final Command commandPivotDown = m_pivot.pivotDown();
  private SendableChooser<Command> autonomousChooser = new SendableChooser<Command>();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(IOConstants.kDriverControllerPort);
  XboxController m_driverControllerSP = new XboxController(IOConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(IOConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    NamedCommands.registerCommand("intake", commandIntake);
    NamedCommands.registerCommand("outtake", commandOuttake);
    NamedCommands.registerCommand("pivotToSpeaker", commandPivotToSpeaker);
    NamedCommands.registerCommand("shoot", commandShoot);
    NamedCommands.registerCommand("idle", commandIdle);
    NamedCommands.registerCommand("pivotDown", commandPivotDown);
    // Configure the trigger bindings
    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    configureDriverBindings();
    globalEventList();
    launchCommands();
  }
  private final PathPlannerAuto m_testAuto1 = new PathPlannerAuto("Test Auto #1");
  private final PathPlannerAuto m_testAuto2 = new PathPlannerAuto("Test Auto #2");

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

    Lock lockMode = new Lock(m_robotDrive, m_pivot, m_camera, moveForward, moveSideways);

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
     * right stick button on controller controls the re-zeroing of the heading
     */

    m_driverController
        .rightStick()
        .onTrue(Commands.runOnce(() -> {
          m_robotDrive.zeroHeading();
        }, m_robotDrive));

    /**
     * INTAKE and FEEDER controls
     * 
     * LEFT BUMPER: Intake note and feed into shooter
     * RIGHT BUMPER: Spit note and outtake
     */
    m_driverController.leftBumper().whileTrue(m_feeder.feedNote().alongWith(m_intake.intake()));

    m_driverController.rightBumper().whileTrue(m_feeder.spitNote().alongWith(m_intake.outtake()));
    
    m_operatorController.rightBumper().whileTrue(m_pivot.pivotToSpeaker()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.FEED)));

    /**
     * SHOOTER PIVOT controls 
     * 
     * LEFT TRIGGER: Pivot up
     * RIGHT TRIGGER: Pivot down
     */
       m_driverController
        .leftTrigger()
        .whileTrue(Commands.runEnd(() -> {
          m_pivot.pivot(PivotDirection.UP);
        }, () -> {
          m_pivot.pivot(PivotDirection.STOP);
        }, m_pivot));

    m_driverController
        .rightTrigger()
        .whileTrue(Commands.runEnd(() -> {
          m_pivot.pivot(PivotDirection.DOWN);
        }, () -> {
          m_pivot.pivot(PivotDirection.STOP);
        }, m_pivot));

        m_operatorController
        .leftTrigger()
        .whileTrue(Commands.runEnd(() -> {
          m_pivot.pivot(PivotDirection.UP);
        }, () -> {
          m_pivot.pivot(PivotDirection.STOP);
        }, m_pivot));

    m_operatorController
        .rightTrigger()
        .whileTrue(Commands.runEnd(() -> {
          m_pivot.pivot(PivotDirection.DOWN);
        }, () -> {
          m_pivot.pivot(PivotDirection.STOP);
        }, m_pivot));

    /**
     * PIVOT auto commands
     * 
     * LEFT D-PAD: Pivot to AMP
     * RIGHT D-PAD: Pivot to Speaker
     */
    m_driverController.povLeft().whileTrue(m_pivot.pivotToAMP()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.IDLE)));

    m_driverController.povRight().whileTrue(m_pivot.pivotToSpeaker()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.SPEAKER)));

    
     m_operatorController.povLeft().whileTrue(m_pivot.pivotToAMP()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.IDLE)));

    m_operatorController.povRight().whileTrue(m_pivot.pivotToSpeaker()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.SPEAKER)));

    /**
     * ShOOTING controls
     * 
     * B BUTTON: Range Shooter Pivot
     * Y BUTTON: Toggle Shooter State
     * A BUTTON: Shoot
     */
    m_driverController.b().whileTrue(m_pivot.pivotToRange()).onTrue(Commands.runOnce(()-> m_shooter.setStateSpeaker(ShooterState.SPEAKER)));

    m_driverController
        .y()
        .onTrue(Commands.runOnce(() -> {
          m_shooter.toggleState(ShooterState.SPEAKER);
        }, m_shooter).andThen(Commands.runOnce(() -> {
          lockMode.setMode(LockMode.SPEAKER_LOCK_MODE);
        })));
    
    m_operatorController
        .y()
        .onTrue(Commands.runOnce(() -> {
          m_shooter.toggleState(ShooterState.SPEAKER);
        }, m_shooter).andThen(Commands.runOnce(() -> {
          lockMode.setMode(LockMode.SPEAKER_LOCK_MODE);
        })));

    m_driverController.a().whileTrue(m_feeder.shootNote());


    /**
     * CLIMBING controls
     * 
     * UP D-PAD: Climb up
     * DOWN D-PAD: Climb down
     */
    // m_driverController.povUp().whileTrue(m_climber.climbUp());
    m_driverController.povUp().whileTrue(m_climber.climbUp());
    m_driverController.povDown().whileTrue(m_climber.climbDown());

    m_operatorController.povUp().whileTrue(m_climber.climbUp());
    m_operatorController.povDown().whileTrue(m_climber.climbDown());

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

  public void getRumble(){
    if(m_feeder.hasNote() == true){
      m_driverControllerSP.setRumble(RumbleType.kLeftRumble, 0.1);
    }
    else if (m_feeder.hasNote() ==  false){
      m_driverControllerSP.setRumble(RumbleType.kLeftRumble, 0);
    }

    if (m_shooter.getSpeed() > 0.5){
      m_driverControllerSP.setRumble(RumbleType.kRightRumble, 0.2);
    }

    else if (m_shooter.getSpeed() < 0.5){
      m_driverControllerSP.setRumble(RumbleType.kRightRumble, 0);
    }
  }

  public void setUpAutonomousDashboard(){
  this.autonomousChooser.setDefaultOption("Shoot back up Basic Auto", m_shootBackUp);
  this.autonomousChooser.addOption("Two Shoot Basic Auto", m_twoShootBasicAuto);
  this.autonomousChooser.addOption("Back Up", m_backUp);
  this.autonomousChooser.addOption("Shoot No Back Up", m_shootNoBackUp);
  this.autonomousChooser.addOption("Three Shoot Left Auto", m_threeShootLeftAuto);
  this.autonomousChooser.addOption("Three Shoot Right Auto", m_threeShootRightAuto);
  this.autonomousChooser.addOption("Front Four Left - Right Auto", m_frontFourLeftToRightAuto);
  this.autonomousChooser.addOption("Front Four Right - Left Auto", m_frontFourRightToLeftAuto);
  this.autonomousChooser.addOption("Test Auto #1", m_testAuto1);
  this.autonomousChooser.addOption("Test Auto #2", m_testAuto2);
  SmartDashboard.putData("Autonomous Chooser", this.autonomousChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //System.out.println("Auton Selected" + AutoChooser.getAuton().getName());
    //return AutoChooser.getAuton();
    // PathPlannerPath path = PathPlannerPath.fromPathFile("example");
    return autonomousChooser.getSelected();

    /* Shoot and back up */
    // return new SequentialCommandGroup(m_pivot.pivotToSpeaker().withTimeout(2.5).alongWith(Commands.runOnce(() -> 
    //   m_shooter.toggleState(ShooterState.SPEAKER))).andThen(new WaitCommand(0.9)).andThen(m_feeder.shootNote().withTimeout(1.0)).andThen(Commands.runOnce(()-> m_shooter.toggleState(ShooterState.IDLE))).andThen(Commands.run(()-> m_robotDrive.drive(-1,0, 0, false), m_robotDrive).alongWith(m_intake.intake()).withTimeout(1.0)));

    //return new PathPlannerAuto("Test Auto #2");
    //PathPlannerPath path = PathPlannerPath.fromPathFile("1 meter forward");

    //Pose2d initialPose = path.getPathPoses().get(0);

    //SmartDashboard.putString("initialPose", initialPose.toString());
    
    //m_robotDrive.resetOdometry(initialPose);

    //return AutoBuilder.followPath(path);
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

  public void launchCommands() {
  }
  
}