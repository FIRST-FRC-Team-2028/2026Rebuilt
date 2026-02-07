// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.opencv.core.Mat;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.PathPlannerLogging;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.VPose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.RobotConstants;

public class Drivetrain extends SubsystemBase {
  static double kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static double kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;
  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;
  private AprilTags aprilSubsystem;

  private final SwerveModule m_frontLeft =
      new SwerveModule(
          "FL",
          Constants.DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftAbsoluteEncoderPort);
  private final SwerveModule m_frontRight =
      new SwerveModule(
          "FR",
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightAbsoluteEncoderPort);
  private final SwerveModule m_backLeft =
      new SwerveModule(
          "BL",
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          DriveConstants.kBackLeftAbsoluteEncoderPort);
  private final SwerveModule m_backRight =
      new SwerveModule(
          "BR",
          DriveConstants.kBackRightDriveMotorPort,
          DriveConstants.kBackRightTurningMotorPort,
          DriveConstants.kBackRightAbsoluteEncoderPort);

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
  private final Pigeon2 m_gyro = new Pigeon2(0);
  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics1,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });
  
  private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),                                                                 
          m_backRight.getPosition()
         },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(20)));

  /** Creates a new Drivetrain. */
  public Drivetrain(AprilTags aprilSubsystem) {
    if (Constants.CAMERA_AVAILABLE){
      this.aprilSubsystem = aprilSubsystem;
    }
    resetGyro();
    for (SwerveModule module : modules) {
      module.resetDriveEncoder();
      module.initializeRelativeTurningEncoder();

    }
    
    //Pathplanner Autobuilder
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder
      AutoBuilder.configure(
        this::getPoseEstimatorPose, 
        this::resetPoseEstimatorPose, 
        this::getChassisSpeeds, 
        this::drive, 
        new PPHolonomicDriveController(
          DriveConstants.translationConstants,
          DriveConstants.rotationConstants
        ),
        config,
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
        this
      );
      //System.out.println("IF I SEE THIS LINE THAT MEANS THAT THE TRY PART OF THE TRY CATCH IS WORKING AND THERE ISN'T AN ERROR INSIDE THE TRY");
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      e.printStackTrace();
    }
    
  
  }
  @Override
  public void periodic() {

    updatePoseEstimator();
    //SmartDashboard.putNumber("front Left Velocity", m_frontLeft.getVelocity());
    
    //SmartDashboard.putNumber("front left abs", m_frontLeft.getAbsTurningPosition(0.1).getDegrees());
    //SmartDashboard.putNumber("front left rel", m_frontLeft.getRelativeTurningPosition().getDegrees());
    //SmartDashboard.putNumber("front right abs", m_frontRight.getAbsTurningPosition(0.1).getDegrees());
    //SmartDashboard.putNumber("front right rel", m_frontRight.getRelativeTurningPosition().getDegrees());
    //SmartDashboard.putNumber("back left abs", m_backLeft.getAbsTurningPosition(0.1).getDegrees());
    //SmartDashboard.putNumber("back left rel", m_backLeft.getRelativeTurningPosition().getDegrees());
    //SmartDashboard.putNumber("back right abs", m_backRight.getAbsTurningPosition(0.1).getDegrees());
    //SmartDashboard.putNumber("back right rel", m_backRight.getRelativeTurningPosition().getDegrees());
    // This method will be called once per scheduler run
  }




  public void BreakMode() {
    m_frontLeft.BreakMode();
    m_frontRight.BreakMode();
    m_backLeft.BreakMode();
    m_backRight.BreakMode();
  }

  public void CoastMode() {
    m_frontLeft.CoastMode();
    m_frontRight.CoastMode();
    m_backLeft.CoastMode();
    m_backRight.CoastMode();
  }

  public void resetGyro(){
    m_gyro.reset();
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }



  /** return a Rotation2d representing the heading of the robot
     * described in radians clockwise from forward
     */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading().getDegrees());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.kPeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    setModuleStates(swerveModuleStates);
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (SwerveModuleState state:desiredStates){
      state.angle= new Rotation2d(-state.angle.getRadians()); //The encoder won't let us invert it 
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
    //SmartDashboard.putNumber("Module Turnin Target", desiredStates[0].angle.getRotations());
  }

  /**Updates the odometry location using swerve module position */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  /**Update the estimate of the robot Pose 
   * based on odometry
   * and any detectable April tags
  */
  public void updatePoseEstimator() {
    m_poseEstimator.update(m_gyro.getRotation2d(),
                          new SwerveModulePosition[] {                                  
                          m_frontLeft.getPosition(),
                          m_frontRight.getPosition(),
                          m_backLeft.getPosition(),
                          m_backRight.getPosition()
    });

    if(Constants.CAMERA_AVAILABLE){
      if (aprilSubsystem.isPoseEstimated()) {
        m_poseEstimator.addVisionMeasurement(
                  aprilSubsystem.getEstimatedPose3d().toPose2d(), aprilSubsystem.estimatedPoseTime); 
      }
      /*if (aprilSubsystem.isPoseEstimated2()) {
        m_poseEstimator.addVisionMeasurement(         FOR SECOND CAMERA
                  aprilSubsystem.getEstimatedPose3d2().toPose2d(), aprilSubsystem.estimatedPoseTime2);
      }*/
          
    }
    SmartDashboard.putNumber("Robot X Pos", m_poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Y Pos", m_poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot Degrees Rotation", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
    };
  }
  
  /** Position of robot with x and y in meters */
  public Pose2d getOdomentryPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdomentryPose(Pose2d pose) {
    //System.out.println(pose);
    m_odometry.resetPosition(getHeading(), getModulePositions(), pose);
  } 

  public Pose2d getPoseEstimatorPose() {
    return m_poseEstimator.getEstimatedPosition();
  }
  public void resetPoseEstimatorPose(Pose2d pose) {
    m_poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }


  public VPose2d getVecToHub(Optional<Alliance> alliance){
    VPose2d whereIam = new VPose2d(getPoseEstimatorPose());
    VPose2d diff = new VPose2d(getPoseEstimatorPose()); //Can't be null
    if(alliance.get() == Alliance.Red){diff = FieldConstants.VPose2dRedHub.minus(whereIam);}
    if (alliance.get() == Alliance.Blue) {diff = FieldConstants.VPose2dBlueHub.minus(whereIam);}
    
    return diff;
  }

  Pose2d wheretoDrive = new Pose2d();
  public Pose2d getTorange(Optional<Alliance> alliance, double range){
    VPose2d whereIam = new VPose2d(getPoseEstimatorPose());
    VPose2d diff = getVecToHub(alliance);
    double dist = diff.norm() - range;
    VPose2d whereToGo = whereIam.plus(diff.unit().scalarProd(dist));
    double theta = Units.radiansToDegrees(Math.atan(diff.Y()/diff.X()));
    if (alliance.get()==Alliance.Red)theta += 180;
    if(dist<=0){ whereToGo = whereIam;}
    System.out.println("Running");
    SmartDashboard.putNumber("dist", dist);
    SmartDashboard.putNumber("TargetX", whereToGo.X());
    SmartDashboard.putNumber("TargetY", whereToGo.Y());
    SmartDashboard.putNumber("TargetTheta", theta);
    wheretoDrive = new Pose2d(whereToGo.X(), whereToGo.Y(), new Rotation2d(Units.degreesToRadians(theta)));
    return wheretoDrive;
    
  }
  public Pose2d getWhereToDrive(){
    return wheretoDrive;
  }



 /**Contructs and runs a path to the given path name avoiding obsticals outlinned in navgrid.json. Uses the
   * normal constraints of the robot as path constraints.
   * @param pathname The name of the path file in the deploy/pathplanner/paths file.
   * Will only print out "FAIL" if the file name does not exist.
  */
  public Command pathfindToPath(String pathname) {
    PathPlannerPath path;
    try{
      path = PathPlannerPath.fromPathFile(pathname);
      return AutoBuilder.pathfindThenFollowPath(path, PathPlannerConstants.pathConstraints);
    } catch(Exception e){
      e.getStackTrace();
      return new InstantCommand(()->System.out.println("FAIL"));
    }
  }
  /**Contructs and runs a path to the given pose avoiding obsticals outlinned in navgrid.json
   * @param x The x cordinate of the target position 
   * @param y The y cordinate of the target position
   * @param rotation the Rotation 2d value of the target position in degrees
   * @param goalEndVelocity The velocity of the robot at the end of the path. 0 is required to stop at the target pose.
   * A value > 0 may be used to keep the robot up to speed for the driver to take over.
   */
  public Command pathfindToPose(double x, double y, double rotation, double goalEndVelocity) {
    Rotation2d rotation2d = new Rotation2d(Units.degreesToRadians(rotation));
    Pose2d targetPose = new Pose2d(x, y, rotation2d);
    SmartDashboard.putString("Target Pose", targetPose.toString());
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.pathConstraintsTest, goalEndVelocity);
  }
  /** Contructs and runs a path to the given pose avoiding obsticals outlinned in navgrid.json
   * @param goalEndVelocity The velocity of the robot at the end of the path. 0 is required to stop at the target pose.
   * @param alliance The alliance from the driver station 
   * @param range the range from the hub to drive to
   * */
  public Command pathfindToPose(Pose2d targetPose, double goalEndVelocity) {
    SmartDashboard.putString("Target Pose", wheretoDrive.toString());
    return AutoBuilder.pathfindToPose(targetPose, PathPlannerConstants.pathConstraintsTest, goalEndVelocity);
  }

  public PathPlannerPath goInRangePath(Optional<Alliance> alliance, double range, double goalEndVelocity){
    Pose2d endPose = getTorange(alliance, range);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        getPoseEstimatorPose(),
        endPose);
    // Create the path using the waypoints created above
      PathPlannerPath path = new PathPlannerPath(
        waypoints,
        PathPlannerConstants.pathConstraintsTest,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(endPose.getRotation().getDegrees())) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
      );
      return path;
  }

}
