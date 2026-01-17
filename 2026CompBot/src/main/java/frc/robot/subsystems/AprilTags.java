// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CamConstants;

public class AprilTags extends SubsystemBase {
  private final PhotonCamera camera;
  AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonPoseEstimator poseEstimator;
  PhotonPipelineResult cameraResult;
  List<PhotonPipelineResult> cameraResults;
  PhotonTrackedTarget target;
  Optional<EstimatedRobotPose> estimatedPose;
  double estimatedPoseTime;
  Pose3d estimatedPose3d;

  /** Creates a new AprilTags. */
  public AprilTags() {
    camera = new PhotonCamera(CamConstants.camera_name);
    aprilTagFieldLayout = aprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, Constants.CamConstants.robot_to_camera);
  }


  @Override
  public void periodic() {
    //cameraResult = camera.getLatestResult();
    cameraResults = camera.getAllUnreadResults();
    estimatedPose = Optional.empty();
    target = cameraResult.getBestTarget();
    for (var result : camera.getAllUnreadResults()) {
            estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
            if (estimatedPose.isEmpty()) {
                estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
            }
      }
      if(estimatedPose.isPresent()){
      EstimatedRobotPose poseEstimated = estimatedPose.get();
      estimatedPoseTime = poseEstimated.timestampSeconds;
      estimatedPose3d = poseEstimated.estimatedPose;
      }
    }
    //estimatedPose = poseEstimator.estimateCoprocMultiTagPose(cameraResult);

    
    // This method will be called once per scheduler run


  public Pose3d getPose3d(){
    return estimatedPose3d;
  }
}
