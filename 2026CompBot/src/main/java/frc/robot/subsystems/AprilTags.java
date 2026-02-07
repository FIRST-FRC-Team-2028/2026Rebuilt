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
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CamConstants;

public class AprilTags extends SubsystemBase {
  private final PhotonCamera camera;
  //private final PhotonCamera camera2;

  AprilTagFieldLayout aprilTagFieldLayout;
  private final PhotonPoseEstimator poseEstimator;
  //private final PhotonPoseEstimator poseEstimator2;

  PhotonPipelineResult cameraResult, cameraResult2;
  List<PhotonPipelineResult> cameraResults;
  PhotonTrackedTarget target;
  Optional<EstimatedRobotPose> estimatedPose, estimatedPose2;
  Optional<MultiTargetPNPResult> multiTagResult;
  double distToTarget;
  double estimatedPoseTime, estimatedPoseTime2;
  Pose3d estimatedPose3d, estimatedPose3d2;
  boolean isEstimated, isEstimated2;

  /** Uses the PhotonVision VendorDep to process April Tags
   * <p>Methods<ul>
   * <li>{@code getMag} - Gets the magnitude of the distance vector with the X and Y components
   * <li>{@code getMagZ} - Gets the magnitude of the distance vector with the X, Y, and Z components
   * <li>{@code isPoseEstimated} - Gets the boolean to determine if a pose is estimated
   * <li>{@code getEstimatedPose3d} - Gets the Pose3d of the position estimated by the pose estimator
   * <li>{@code getTimestamp} - Gets the timestamp of the position estimated the pose estimator
   * </ul>
   * </p>
   */
  public AprilTags() {
    camera = new PhotonCamera(CamConstants.camera_name);
    //camera2 = new PhotonCamera(CamConstants.camera_name2);
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, Constants.CamConstants.robot_to_camera);
    //poseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, Constants.CamConstants.robot_to_camera2);

  }


  @Override
  public void periodic() {
    for (var results: camera.getAllUnreadResults()){
       estimatedPose = poseEstimator.estimateCoprocMultiTagPose(results);
      if (estimatedPose.isPresent()){
        estimatedPoseTime = estimatedPose.get().timestampSeconds;
        estimatedPose3d = estimatedPose.get().estimatedPose;
        isEstimated=true;
      } else if (estimatedPose.isPresent()) {  //If multi tag isn't present, use average best targets
        estimatedPose = poseEstimator.estimateAverageBestTargetsPose(results);
        estimatedPoseTime = estimatedPose.get().timestampSeconds;
        estimatedPose3d = estimatedPose.get().estimatedPose;
        isEstimated=true;
      } else isEstimated = false;
    }

    /*for (var results2: camera2.getAllUnreadResults()){
       estimatedPose2 = poseEstimator2.estimateCoprocMultiTagPose(results2);
      if (estimatedPose2.isPresent()){
        estimatedPoseTime2 = estimatedPose2.get().timestampSeconds;
        estimatedPose3d2 = estimatedPose2.get().estimatedPose;
        isEstimated2 = true;  TODO 2nd Camera?
      } else if (estimatedPose2.isPresent()) {  //If multi tag isn't present, use average best targets
        estimatedPose2 = poseEstimator2.estimateAverageBestTargetsPose(results2);
        estimatedPoseTime2 = estimatedPose2.get().timestampSeconds;
        estimatedPose3d2 = estimatedPose2.get().estimatedPose;
        isEstimated2=true;
      } else isEstimated2 = false;
    }*/
    
  }
  /** Gets the boolean to determine if a pose is estimated
   * @return If a pose is estimated
   */
  public boolean isPoseEstimated(){
    return isEstimated;
  }

  /** Gets the Pose3d of the position estimated by the pose estimator
   * @return estimatedPose3d: The Pose3d of the position estimated by the pose estimator
   */
  public Pose3d getEstimatedPose3d(){
    return estimatedPose3d;
  }

  /** Gets the timestamp of the position estimated the pose estimator 
   * @return estimatedPoseTime: The timestamp of the position estimated the pose estimator 
  */
  public double getTimeStamp(){
    return estimatedPoseTime;
  }
  /*public Pose3d getEstimatedPose3d2(){
    return estimatedPose3d2;
  }
  public double getTimeStamp2(){  TODO 2nd Camera?
    return estimatedPoseTime2;
  }*/
  
  /** Gets the magnitude of the distance vector with the X, Y, and Z components
   * @param target the target that the distance is found
   * @return the magnitude of the distance vector to the target AprilTag
   */
  public double getMagZ(PhotonTrackedTarget target) {
    var xdist2 = Math.pow(target.getBestCameraToTarget().getX(),2);
    var ydist2 = Math.pow(target.getBestCameraToTarget().getY(),2);
    var zdist2 = Math.pow(target.getBestCameraToTarget().getZ(),2);
    return Math.sqrt(xdist2+ydist2+zdist2);
  }

  /** Gets the magnitude of the distance vector with the X, and Y components
   * @param target the target that the distance is found
   * @return the magnitude of the distance vector to the target AprilTag
   */
    public double getMag(PhotonTrackedTarget target) {
    var xdist2 = Math.pow(target.getBestCameraToTarget().getX(),2);
    var ydist2 = Math.pow(target.getBestCameraToTarget().getY(),2);
    return Math.sqrt(xdist2+ydist2);
    }
}
