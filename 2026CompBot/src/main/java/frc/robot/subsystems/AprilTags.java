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
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  double distToTarget;
  double estimatedPoseTime;
  Pose3d estimatedPose3d;
  double xdist2,ydist2,zdist2;

  /** Creates a new AprilTags. */
  public AprilTags() {
    camera = new PhotonCamera(CamConstants.camera_name);
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, Constants.CamConstants.robot_to_camera);
  }


  @Override
  public void periodic() {
    //cameraResult = camera.getLatestResult();
    cameraResults = camera.getAllUnreadResults();
    estimatedPose = Optional.empty();
    //target = cameraResult.getBestTarget();
    for (var result : camera.getAllUnreadResults()) {
      if(result.hasTargets()){
        target = result.getBestTarget();
        distToTarget = getNorm(target);
        SmartDashboard.putNumber("Distance To Target", distToTarget);
        SmartDashboard.putNumber("Distance To Target (X only)", target.getBestCameraToTarget().getX());
        estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);
        if (estimatedPose.isEmpty()){
          estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
        }
    }
      if(estimatedPose.isPresent()){
      EstimatedRobotPose poseEstimated = estimatedPose.get();
      estimatedPoseTime = poseEstimated.timestampSeconds;
      estimatedPose3d = poseEstimated.estimatedPose;
      }
    }
  }
    
  public Pose3d getPose3d(){
    return estimatedPose3d;
  }

  public double getNorm(PhotonTrackedTarget target) {
    xdist2 = Math.pow(target.getBestCameraToTarget().getX(),2);
    ydist2 = Math.pow(target.getBestCameraToTarget().getY(),2);
    zdist2 = Math.pow(target.getBestCameraToTarget().getZ(),2);
    return Math.sqrt(xdist2+ydist2+zdist2);
  }


}
