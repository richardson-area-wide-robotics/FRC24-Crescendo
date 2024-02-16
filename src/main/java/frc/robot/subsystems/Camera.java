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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  PhotonCamera camera;
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  PhotonPoseEstimator photonPoseEstimator; 
  //meters
  final double cameraOffsetX = -0.33655;
  final double cameraOffsetY = -0.01016;
  final double cameraOffsetZ = 0.0889;
  //radians
  final double cameraRoll = 0;
  final double cameraPitch = 0;
  final double cameraYaw = 0;
  Transform3d cameraToRobot;

  public Camera(String name) {
    this.camera = new PhotonCamera(name);
    this.cameraToRobot = new Transform3d(new Translation3d(cameraOffsetX, cameraOffsetY, cameraOffsetZ), new Rotation3d(cameraRoll, cameraPitch, cameraYaw));
    this.photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, cameraToRobot);
  }

  @Override
  public void periodic() {
    //System.out.println(getTagID());
    //System.out.println(getTransforms());
    //System.out.println(getEstimatedGlobalPose());

    final Optional<EstimatedRobotPose> robotPose = getEstimatedGlobalPose();
    if (robotPose.isPresent()) {
    System.out.println(robotPose.get().estimatedPose);
  }
}

  /**
   * Returns a list of the fiducial IDs of all the AprilTags currently being detected, in an arbitrary order
   * Returns an empty list if no targets are found
   * @return
   */
  public List<Integer> getTagID() {
    PhotonPipelineResult result = camera.getLatestResult();
    ArrayList<Integer> targetIds = new ArrayList<Integer>();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (int i = 0; i < targets.size(); i++) {
        targetIds.add(targets.get(i).getFiducialId());
      }
    }
    return targetIds;
  }
/**
 * Returns a list of the distances of all visible AprilTags from the camera in meters
 * Returns an empty list if no targets are found
 * @return
 */
  public List<Transform3d> getTransforms() {
    PhotonPipelineResult result = camera.getLatestResult();
    ArrayList<Transform3d> targetTransforms = new ArrayList<Transform3d>();
    if (result.hasTargets()) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (int i = 0; i < targets.size(); i++) {
        targetTransforms.add(targets.get(i).getBestCameraToTarget());
      }
    }
    return targetTransforms;
  }

  /**
   * Returns the global position of the robot.
   * @return
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return photonPoseEstimator.update();
  }

  public double GetAngleToSpeaker()
  {
    
    final Optional<EstimatedRobotPose> robotPose = getEstimatedGlobalPose();
    if (robotPose.isPresent()) {
    
    Pose3d speakerPose3d = aprilTagFieldLayout.getTagPose(3).get();
      Transform3d speakerToRobot = speakerPose3d.minus(robotPose.get().estimatedPose);

      // speakerToRobot.getRotation().getY
    }

    return 0.0;
  
  }
}
