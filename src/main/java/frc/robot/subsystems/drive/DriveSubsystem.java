// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.swerve.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;

public class DriveSubsystem extends Swerve {

  static final MAXSwerveModule frontLeft = new MAXSwerveModule(
      Constants.SwerveDriveConstants.FrontLeftModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule frontRight = new MAXSwerveModule(
      Constants.SwerveDriveConstants.FrontRightModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule backLeft = new MAXSwerveModule(
      Constants.SwerveDriveConstants.BackLeftModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule backRight = new MAXSwerveModule(
      Constants.SwerveDriveConstants.BackRightModule.S_MODULE_CONSTANTS);

  private Camera m_camera;

   /*public DriveSubsystem() {
    super(
        frontLeft,
        frontRight,
        backLeft,
        backRight,
        Constants.SwerveDriveConstants.kDriveKinematics,
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        m_gyro,
        Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements*/
    

  public DriveSubsystem(AHRS m_gyro, Camera camera) {
    super(
        frontLeft,
        frontRight,
        backLeft,
        backRight,
        Constants.SwerveDriveConstants.kDriveKinematics,
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        m_gyro,
        Constants.SwerveDriveConstants.kMaxSpeedMetersPerSecond);

    // AutoBuilder.configureHolonomic(
    //     this::getPose, // robot pose supplier
    //     this::resetOdometry, // Method to reset odometry
    //     this::getChassisSpeeds,
    //     this::driveRobotRelative,
    //     Constants.AutoConstants.kPathFollowerConfig, () -> {
    //       // Boolean supplier that controls when the path will be mirrored for the red
    //       // alliance
    //       // This will flip the path being followed to the red side of the field.
    //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE


    //       var alliance = DriverStation.getAlliance();
    //       if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //       }
    //       return false;
    //     }, this);

    AutoBuilder.configureHolonomic(
      this::getPose, // Robot pose supplier
      this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.4, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig() // Default path replanning config. See the API for the options here
      ),
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this); // Reference to this subsystem to set requirements


    this.m_camera = camera;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("X Controller", Constants.AutoConstants.kPXController);
    addChild("Y Controller", Constants.AutoConstants.kPYController);
    addChild("Theta Controller", Constants.AutoConstants.kPThetaController);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> pose = m_camera.getEstimatedGlobalPose();
    Optional<Double> time = m_camera.getPoseTimeStamp();

    SmartDashboard.putBoolean("pose.isPresent()", pose.isPresent());
    SmartDashboard.putBoolean("time.ispresent", time.isPresent());
    if (pose.isPresent() && time.isPresent()) {
      this.addPoseEstimate(pose.get().estimatedPose.toPose2d(), time.get());
    }

    SmartDashboard.putString("swervepose", this.getPose().toString());
    super.periodic();
  }
}