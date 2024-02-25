// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Distance;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Angle;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import frc.lib.controller.FFGains;
import frc.lib.controller.PIDGains;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  // public static class ShooterConstants {
  //   public static final int kickerMotorCANID = 12;
  //   public static final int shooterRightCANID = 13;
  //   public static final int shooterLeftCANID = 14;

  //   public static final int kickerMotorCurrentLimit = 60;
  //   public static final int shooterRightMotorCurrentLimit = 60;
  //   public static final int shooterLeftMotorCurrentLimit = 60;

  //   public static final double kShooterP = 0.0007;
  //   public static final double kShooterI = 0.0;
  //   public static final double kShooterD = 0.001;

  //   public static final Measure<Angle> kPivotToleranceAngle = Degrees.of(3.6);

  //   // smallest angle (between hardstop and shooter) that the shooter can pivot to
  //   public static final Measure<Angle> kPivotMinAngle = Degrees.of(5.0); // TBD

  //   // highest angle (between hardstop and shooter) that the shooter can pivot to
  //   public static final Measure<Angle> kPivotMaxAngle = Degrees.of(45.0); // TBD

  //   public static final boolean kShooterLeftMotorInverted = false;
  //   public static final boolean kShooterRightMotorInverted = true;

  //   public static final Measure<Distance> kHorizontalNoteCompression = Inches.of(12.0);
  //   public static final Measure<Distance> kShooterWheelRadius = Inches.of(2.0);
  //   public static final Measure<Velocity<Angle>> kKickerSpeed = RPM.of(300.0);

  //   public static final Measure<Velocity<Distance>> kLaunchSpeedTolerance = MetersPerSecond.of(0.01);

  //   public static enum PivotDirection {
  //     UP, 
  //     DOWN,
  //     STOP
  //   }

  //   public static final int kAmpAngleDegrees = 90;


  //   public static enum ShooterState {
  //     IDLE, 
  //     INTAKE,
  //     OUTTAKE,
  //     SPEAKER,
  //     AMP,
  //     REVERSE
  //   }
  // }

  // public static final class Intake {
  //   public static final int kIntakeMotorPort = 9;
  //   public static final boolean kIntakeMotorInverted = true;
  //   public static final int kIntakeCurrentLimit = 40;
  //   public static final int kIntakeSensorPort = 0;
  //   public static final IdleMode kIntakeIdleMode = IdleMode.kCoast;

  //   public static final boolean kFeederMotorInverted = false;
  //   public static final int kFeederCurrentLimit = 60;

  //   public static final int kIntakeCANID = 15;
  //   public static final int kFeederCANID = 11;

  //   public static final double intakeSpeed = 1;
  //   public static final double feederSpeed = 1;

  //   public static enum IntakeState {
  //     IDLE, 
  //     INTAKE,
  //     OUTTAKE,
  //     FIRE
  //   }
  // }

  public static final class IOConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kControllerDeadband = 0.1;
  }

  public static final class SwerveDriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    /*
     * Chassis configuration
     */
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // The kinematics for the robot drivetrain
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // DO NOT RUN WITHOUT CHECKING THIS
    public static final double kDriveRadius = Math.sqrt(Math.pow(kWheelBase, 2) + Math.pow(kTrackWidth, 2));

    /*
     * Spark Max and encoder constents for MAXSwerve modules
     */
    public static final class FrontLeftModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 5;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 6;
      // The Angular offset in radians for the steer encoder in radians
      public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
      // creating the swerve module constants for the front left module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS = new SwerveModuleConstants(
          kDriveMotorCANID, kSteerMotorCANID, kFrontLeftChassisAngularOffset);
    }

    public static final class FrontRightModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 7;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 8;
      // The Angular offsets in radians for the steer encoder
      public static final double kFrontRightChassisAngularOffset = 0;
      // creating the swerve module constants for the front right module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS = new SwerveModuleConstants(
          kDriveMotorCANID, kSteerMotorCANID, kFrontRightChassisAngularOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class BackLeftModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 3;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 4;
      // The Angular offsets in radians for the steer encoder
      public static final double kBackLeftChassisAngularOffset = Math.PI;
      // creating the swerve module constants for the back left module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS = new SwerveModuleConstants(
          kDriveMotorCANID, kSteerMotorCANID, kBackLeftChassisAngularOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class BackRightModule {
      // The CAN ID for the drive motor
      public static final int kDriveMotorCANID = 1;
      // The CAN ID for the steer motor
      public static final int kSteerMotorCANID = 2;
      // The Angular offsets in radians for the steer encoder
      public static final double kBackRightChassisAngularOffset = Math.PI / 2;
      // creating the swerve module constants for the back right module
      public static final SwerveModuleConstants S_MODULE_CONSTANTS = new SwerveModuleConstants(
          kDriveMotorCANID, kSteerMotorCANID, kBackRightChassisAngularOffset);
    }

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    public static final boolean kDrivingIsInverted = false;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kFreeSpeedRpm = 5676.0;
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60.0;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15.0);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0.0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final PIDGains kDrivingPIDGains = new PIDGains(0.04, 0, 0); // TODO: tune values for the driving motor
    public static final FFGains kDrivingFFGains = new FFGains(0, 0, 0);
    public static final double kDrivingFF = 1.0 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1.0;
    public static final double kDrivingMaxOutput = 1.0;

    // TODO: tune values for the turning motor
    public static final PIDGains kTurningPIDGains = new PIDGains(3.15, 0, 0.35);
    public static final FFGains kTurningFFGains = new FFGains(0, 0, 0);
    // TODO: tune values for Feed Forward
    public static final double kTurningFF = 0.0;
    public static final double kTurningMinOutput = -1.0;
    public static final double kTurningMaxOutput = 1.0;

    // TODO: tune values for Vision auto-turning
    public static final PIDGains kVisionTurningPIDGains = new PIDGains(1.0, 0, 0.01);

    public static final double MAX_LOCKED_ON_SPEED = 0.33;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 10; // amps
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final PIDController kPXController = new PIDController(1.0, 0, 0.35);
    public static final PIDController kPYController = new PIDController(0.001, 0, 0.00075);
    public static final PIDController kPThetaController = new PIDController(8.0, 0, 0.75);

    // PID constants for new Pathplanner code
    // TODO: tune these values
    public static final PIDConstants kTranslationGains = new PIDConstants(.05, 0.0, 0.0);
    public static final PIDConstants kRotationGains = new PIDConstants(3, 0.0, .35);

    public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig(
        kTranslationGains, kRotationGains, kMaxSpeedMetersPerSecond, SwerveDriveConstants.kDriveRadius,
        new ReplanningConfig());

    // Drive base radius in meters. Distance from robot center to furthest module.
    public static final double kDriveBaseRadius = 0.48;

    public static final PIDGains kMovingPIDGains = new PIDGains(0.3, 0, 0.01);
    public static final double offset = 0.00315;
  }

  public static final boolean kCompetitionMode = false;


  /* Pivot Constants */
    public static final class PivotConstants{
    // Id's for the pivot motors
    public static final int pivotRightCANID = 9;
    public static final int pivotLeftCANID = 10;

    // Controller idle mode and current
    public static final IdleMode pivotIdleMode = IdleMode.kBrake;
    public static final int pivotCurrentLimit = 40;
    public static final boolean pivotRightMotorInverted = true;

    // TODO: Tune these values for the pivot please 
    public static final double kPivotP = 3.0;
    public static final double kPivotI = 0.00008;
    public static final double kPivotD = 0.45;
    public static final boolean kPivotPositionPIDWrappingEnabled = false;
    public static final double kPivotMinOutput = -1.0;
    public static final double kPivotMaxOutput = 1.0;
    
    // Range of values the smart controller will use as a hard stop 
    public static final float kPivotForwardSoftLimit = 0.31f;
    public static final float kPivotReverseSoftLimit = 0.01f;

    // Range that the driver will be allowed to move pivot to
    public static final double kPivotMaxAngle = 0.3; 
    public static final double kPivotMinAngle = 0.01;
   
    public static final double kPivotPresetSubwoofer = 0.135;
    public static final double kPivotPresetAmp = 0.31;

    public static final double kPivotSpeed = 0.85;
    }






   public static class ShooterConstants {




    public static final int kickerMotorCANID = 12;
    public static final int shooterRightCANID = 13;
    public static final int shooterLeftCANID = 14;

    public static final int kickerMotorCurrentLimit = 60;
    public static final int shooterRightMotorCurrentLimit = 60;
    public static final int shooterLeftMotorCurrentLimit = 60;
    public static final int pivotRightMotorCurrentLimit = 40;
    public static final int pivotLeftMotorCurrentLimit = 40;

    public static final double kPivotP = 1.5;
    public static final double kPivotI = 0.0;
    public static final double kPivotD = 0.001;

    public static final double kShooterP = 0.0007;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.001;

    public static final double kKickerP = 0.0;
    public static final double kKickerI = 0.0;
    public static final double kKickerD = 0.0;

    public static final boolean pivotRightMotorInverted = true;

    public static final Measure<Angle> kPivotToleranceAngle = Degrees.of(3.6);

    // smallest angle (between hardstop and shooter) that the shooter can pivot to
    public static final Measure<Angle> kPivotMinAngle = Degrees.of(5.0); // TBD

    // highest angle (between hardstop and shooter) that the shooter can pivot to
    public static final Measure<Angle> kPivotMaxAngle = Degrees.of(45.0); // TBD

    public static final boolean kShooterLeftMotorInverted = true;
    public static final boolean kShooterRightMotorInverted = false;

    public static final Measure<Distance> kHorizontalNoteCompression = Inches.of(12.0);
    public static final Measure<Distance> kShooterWheelRadius = Inches.of(2.0);
    public static final Measure<Velocity<Angle>> kKickerSpeed = RPM.of(300.0);

    public static final Measure<Velocity<Distance>> kLaunchSpeedTolerance = MetersPerSecond.of(0.01);

    public static final double kPivotSpeed = 0.22;

    public static enum PivotDirection {
      UP, 
      DOWN,
      STOP
    }

    public static final int kAmpAngleDegrees = 90;


    public static enum ShooterState {
      IDLE, 
      INTAKE,
      OUTTAKE,
      SPEAKER,
      AMP,
      REVERSE
    }
  }

  public static final class Intake {
    public static final int kIntakeMotorPort = 15;
    public static final boolean kIntakeMotorInverted = true;
    public static final int kIntakeCurrennLimit = 60;
    public static final int kIntakeSensorPort = 0;
    public static final IdleMode kIntakeIdleMode = IdleMode.kCoast;
    public static final int kFeederCurrentLimit = 60;

    // public static final int intakeCANID = 15;
    public static final int feederCANID = 11;

    public static final double intakeSpeed = 1;
    public static final double feederSpeed = 1;

    public static enum IntakeState {
      IDLE, 
      INTAKE,
      OUTTAKE,
      FIRE
    }
  }

}
