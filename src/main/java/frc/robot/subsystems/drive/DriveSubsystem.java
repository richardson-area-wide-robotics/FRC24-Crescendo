// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.Swerve;
import frc.robot.Constants;

public class DriveSubsystem extends Swerve {
  // public Command trajectoryFollowerCommand(PathPlannerTrajectory trajectory) {
  //   Constants.AutoConstants.kPThetaController.enableContinuousInput(-Math.PI, Math.PI);
  //   return trajectoryFollowerCommand(
  //       trajectory,
  //       Constants.AutoConstants.kPXController,
  //       Constants.AutoConstants.kPYController,
  //       Constants.AutoConstants.kPThetaController);
  // }

  static final MAXSwerveModule frontLeft =
      new MAXSwerveModule(Constants.SwerveDriveConstants.FrontLeftModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule frontRight =
      new MAXSwerveModule(Constants.SwerveDriveConstants.FrontRightModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule backLeft =
      new MAXSwerveModule(Constants.SwerveDriveConstants.BackLeftModule.S_MODULE_CONSTANTS);

  static final MAXSwerveModule backRight =
      new MAXSwerveModule(Constants.SwerveDriveConstants.BackRightModule.S_MODULE_CONSTANTS);

  public DriveSubsystem(AHRS m_gyro) {
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
    //   this::getPose, // robot pose supplier 
    //   this::resetOdometry, // Method to reset odometry
    //   this::getChassisSpeeds, 
    //   this::driveRobotRelative, 
    //   new HolonomicPathFollowerConfig(
    //     Constants.AutoConstants.kTranslationGains, 
    //     Constants.AutoConstants.kRotationGains, 
    //     null, getAngle()), null);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("X Controller", Constants.AutoConstants.kPXController);
    addChild("Y Controller", Constants.AutoConstants.kPYController);
    addChild("Theta Controller", Constants.AutoConstants.kPThetaController);
  }
}