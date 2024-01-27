// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  PhotonCamera camera;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, null);

  public Camera(String name) {
    this.camera = new PhotonCamera(name);
  }

  @Override
  public void periodic() {
    System.out.println(getTagID());
    // System.out.println(getEstimatedGlobalPose());
  }

  /**
   * Returns a list of the fiducial IDs of all the AprilTags currently being detected, in an arbitrary order
   * Returns an empty list if no targets are found
   * @return
   */
  public List<Integer> getTagID() {
    PhotonPipelineResult result = camera.getLatestResult();
    ArrayList<Integer> targetIds = new ArrayList<Integer>();
    boolean hasTargets = result.hasTargets();
    if (hasTargets) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (int i = 0; i < targets.size(); i++) {
        targetIds.add(targets.get(i).getFiducialId());
      }
    }
    return targetIds;
  }

  /**
   * Returns the global position of the robot.
   * @return
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    throw new UnsupportedOperationException("Not implemented");
    //return photonPoseEstimator.update();
  }
}
