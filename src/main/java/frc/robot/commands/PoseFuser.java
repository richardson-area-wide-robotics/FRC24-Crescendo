// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.Swerve;
import frc.robot.subsystems.Camera;

/** An example command that uses an example subsystem. */
public class PoseFuser extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Camera m_camera;
  private final Swerve m_swerve;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PoseFuser(Camera camera, Swerve swerve) {
    m_camera = camera;
    m_swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putBoolean("updated Pose" , false);
    Optional<EstimatedRobotPose> pose = m_camera.getEstimatedGlobalPose();
    Optional<Double> time = m_camera.getPoseTimeStamp();
    if (pose.isPresent() && time.isPresent()) {
      m_swerve.addPoseEstimate(pose.get().estimatedPose.toPose2d(), time.get());
    SmartDashboard.putBoolean("updated Pose" , true);
    }

    SmartDashboard.putString("pose" , m_swerve.getPose().toString());
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
