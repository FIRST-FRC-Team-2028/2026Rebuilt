// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/** Table of Contents
 *  Availbilty of Subsystems
 *  ModuleConstants
 *  DriveConstants
 *  IntakeConstants
 *  ShooterConstants
 *  ClimberConstants
 *  CANIDS
 *  OIConstants
 *  CamConstants
 *  RobotConstants
 *  PathPlannerConstants
 *  FieldConstants
 */
public final class Constants {
  public static final boolean DRIVE_AVAILABLE = false;
  public static final boolean CAMERA_AVAILABLE = false;
  public static final boolean SHOOTER_AVAILABLE = false;
  public static final boolean INTAKE_AVAILABLE = true;
  public static final boolean CLIMBER_AVAILABLE = false;
  public static final boolean PIXYCAM_AVAILABLE = false;



  public static final class ModuleConstants {
     public static final int kDriveMotorCurrentLimit = 80;
    public static final int kTurningMotorCurrentLimit = 80;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
    public static final double kDriveMotorGearRatio = 1 / 5.36; //2026 Competition Robot
    public static final double kTurningMotorGearRatio = 1 / 18.75;  //2026 Competition Robot
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
   // Theoraticlly maybe; in practice from measurements
    //public static final double kDriveEncoderRot2Meter = Units.inchesToMeters(1)/41.2;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 42;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 42;
    public static final double kPTurning_Comp = 0.5;
    public static final double kDriveP = 0.22; // 2025 Competition Robot
    public static final double kDriveI = 0.0; // 2025 Competition Robot
    public static final double kDriveD = 0.0; // 2025 Competition Robot
    public static final double kDriveFF = 0.255; // 2025 Competition Robot


    public static final double kTurningP = 2.3;//2.05; // 2023 Competition Robot   0.75
    public static final double kTurningI = 0.0; // 2025 Competition Robot
    public static final double kTurningD = 0.0; // 2025 Competition Robot

    public static final double kTurnGearRatio = 18.75; // 2025 Competion Robot Mark 4n ratio      12.8 on project X
    public static final double kTurnPositionConversionFactor = 1.0 / kTurnGearRatio;

    // By default, the drive encoder in position mode measures rotations at the drive motor
    // Convert to meters at the wheel
    public static final double kDriveGearRatio = 5.36; // 2025 Competion Robot            6.75 on Project X
    public static final double kDrivePositionConversionFactor =
      (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    // By default, the drive encoder in velocity mode measures RPM at the drive motor
    // Convert to meters per second at the wheel
    public static final double kDriveVelocityConversionFactor =
      kDrivePositionConversionFactor / 60.0;
    public static final double kRampRate = 1;
    public static final double kRampRateT = 0.75; //0.75; 
    public static final double AbsoluteSensorDiscontinuityPoint = 0.5; //1 is value between [0, 1] 0.5 is value between [-0.5, 0.5] 0 is value between [-1, 0] -CTRE Docs
  }

  public static class DriveConstants{
     //Defines the conventional order of the modules when in arrays
        public static final int Front_Left = 0;
        public static final int Front_Right = 1;
        public static final int Back_Left = 2;
        public static final int Back_Right = 3;

        // Distance between right and left wheels
        // Distance between front and back wheels
        public static final double kTrackWidth = Units.inchesToMeters(21.5);      
        //
        public static final double kWheelBase = Units.inchesToMeters(21.5);          
        // 
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final SwerveDriveKinematics kDriveKinematics1 = new SwerveDriveKinematics(
//                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
  //              new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
    //            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      //          new Translation2d(-kWheelBase / 2,  kTrackWidth / 2));

                new Translation2d( kWheelBase / 2,  kTrackWidth / 2), // front left
                new Translation2d( kWheelBase / 2, -kTrackWidth / 2), // front right
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2), // back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right



                  

//    COMP BOT  FRONT                PRAC BOT    FRONT
//     +----------------------+        +----------------------+
//     | D11 S21      D12 S22 |        | D15 S25      D16 S26 |
//     | E31          E32     |        | E35          E36     |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     | D13 S23      D14 S24 |        | D17 S27      D18 S28 |
//     | E33          E34     |        | E37          E38     |
//     +----------------------+        +----------------------+
//
//Front Left
        public static final int kFrontLeftDriveMotorPort        = 10;    // Module 1
        public static final int kFrontLeftTurningMotorPort      = 11;
        public static final int kFrontLeftAbsoluteEncoderPort   = 12;

//Front Right
        public static final int kFrontRightDriveMotorPort       = 20;    // Module 2
        public static final int kFrontRightTurningMotorPort     = 21;
        public static final int kFrontRightAbsoluteEncoderPort  = 22;

//Back Right
        public static final int kBackRightDriveMotorPort        = 30;    // Module 3
        public static final int kBackRightTurningMotorPort      = 31;
        public static final int kBackRightAbsoluteEncoderPort   = 32;

//Back Left
        public static final int kBackLeftDriveMotorPort         = 40;    // Module 4
        public static final int kBackLeftTurningMotorPort       = 41;
        public static final int kBackLeftAbsoluteEncoderPort    = 42;


        //Encoder Inversions
  public static final boolean kFrontLeftTurningEncoderReversed  = true;  //true
  public static final boolean kBackLeftTurningEncoderReversed   = true; //true
  public static final boolean kFrontRightTurningEncoderReversed = true;  //true
  public static final boolean kBackRightTurningEncoderReversed  = true;  //true

  public static final boolean kFrontLeftDriveEncoderReversed  = false;
  public static final boolean kBackLeftDriveEncoderReversed   = false;
  public static final boolean kFrontRightDriveEncoderReversed = false;
  public static final boolean kBackRightDriveEncoderReversed  = false;

  public static final boolean kFrontLeftDriveAbsoluteEncoderReversed  = true;
  public static final boolean kBackLeftDriveAbsoluteEncoderReversed   = true;
  public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
  public static final boolean kBackRightDriveAbsoluteEncoderReversed  = true;

  // Speed Limits
  public static final double kPhysicalMaxSpeedMetersPerSecond = 4.923;  //2026
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * 3 * Math.PI;

  public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond /1.5;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                                     kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.8;

  public static final double kFineControlSpeed = .5;
  public static final double kFasterSpeed = .4;

  public static final String kAbsEncoderMagnetOffsetKey = "kAbsEncoderMagnetOffsetKey";
  public static final double kDefaultAbsEncoderOffset = 0.0;

  // Units are meters per second
  public static final double kMaxTranslationalVelocity = 4.923; // 2026 Competion Robot 

  // Units are radians per second
  public static final double kMaxRotationalVelocity = 5.0; // 2023 Competion Robot // max 5.0
  public static final double kRotateToZero = -2;
  public static final PIDConstants translationConstants = 
    new PIDConstants(6.0, 1.6, 1);     //d:.475
  public static final PIDConstants rotationConstants = 
    new PIDConstants(2.9,0.,0.);       

  }

  public static class IntakeConstants {
    //Configs
    public static final double jointP = 0.01;
    public static final double jointI = 0.;
    public static final double jointD = 0.;
    public static final double JointPositionConversionFactor = 110./38.; //in -20, target in -20| out -58, target out 90
    // We found that the encoder range is more negative out than in, but we cannot set this in the conversion factor
    public static final double JointFPositionConversionFactor = 110./38.; //in -20, target in -20| out -58, target out 90
    // We found that the encoder range is more negative out than in, but we cannot set this in the conversion factor
    public static final double RollerVelocityConversionFactor = 1.;
    public static final double jointForwardSoftLimit = 20.;
    public static final double jointReverseSoftLimit = -90.;
    public static final double jointFForwardSoftLimit = 20.;
    public static final double jointFReverseSoftLimit = -90.;

    //Joint Positions
    public static final double JointPickupPosition = -90.; //Degrees
    public static final double JointUpPosition = 20.; //Degrees
    public static final double JointAgitatePosition = -75.; //Degrees
    public static final double JointFPickupPosition = -90.; //Degrees //TODO change the numbers for the follower motor
    public static final double JointFUpPosition = 20.; //Degrees
    public static final double JointFAgitatePosition = -75.; //Degrees

    //Rollers Speed
    public static final double IntakeSpeed = 0.75; //-1 to 1
  }

  public static class ShooterConstants {
    public static final double shooterP = 0.0002;
    public static final double shooterI = 0.00000015;
    public static final double shooterD = 0.015;
    public static final double shooterVelocityConversionFactor = 1;
    public static final double conveyorP = 0;
    public static final double conveyorI = 0;
    public static final double conveyorD = 0;
    public static final double conveyorVelocityConversionFactor = 1;


    public static final double shooterShootSpeed = 1400.; //RPM
    public static final double feedShootSpeed = 200.; //RPM
    public static final double conveyorShootSpeed = 300.; //RPM
    
    public static final double OptimalRange = 2.5;  //Meters
    public static final double MinRange = 1.5;      //Meters

    //Shooter Math Constants 
    public static final double gravity = 9.81; //m/s^2
    public static final double h_shooter = 17; //height as of 2/18/26
    public static final double delta_H = Units.inchesToMeters(0)-Units.inchesToMeters(h_shooter);// 72 - h_shooter
    public static final double shooter_angle_deg = 50; //Estimate TODO set real angle
    public static final double wheel_diameter_meter = Units.inchesToMeters(4.25);  
    public static final double max_shooting_distance_meters = 0.0; //TODO find max shooting distance
    public static final double velocity_fudge_factor = 1.15; //Typical range: 1.10–1.20

  }

  public static class ClimberConstants {

    public static final double p = 0;
    public static final double i = 0;
    public static final double d = 0;
    public static final double forwardSoftLimit = 0;
    public static final double reverseSoftLimit = 0;
    public static final double positionConversionFactor = 1;
    public static final double kRampRate = 1;
    public static final double climbLevel2 = 0; //position for level1, Degrees
    public static final double climbLevel1 = 0; //position for level1, Degrees
    public static final double hookPosition = 0; //position to engage, Degrees
    public static final double travelPosition = 0;// retracted position, Degrees

  }

  public static class CANIDS {
    //Intake
    public static final int roller = 60;
    public static final int jointLead = 50;
    public static final int jointFollow = 52;
    //Shooter
    public static final int leftShooter = 62;
    public static final int centerShooter = 61;
    public static final int rightShooter = 1000;
    public static final int conveyor = 1000;
    //Climber
    public static final int climber = 70;
    public static final int climber_follower = 71;
    
  }

  public static class OIConstants {
    public static final int kDriverControllerPort =      0;
    public static final int kMechControllerPort =        1; 
    public static final int kMechControllerPort2 =       2;
    //Driver Axis (Includes triggers)
    public static final int kDriverYAxis =               0;
    public static final int kDriverXAxis =               1;
    public static final int kFineControlAxis =           2;
    public static final int kFastControlAxis =           3;
    public static final int kDriverRotAxis =             4;
    //Driver Buttons
    public static final int kResetGyro =                 1;
    public static final int kDriveToShootRange =         3;
    public static final int kDriveToMechPose =           4;

    public static final int kDriverRobotOrientedButton = 6;
    //Game Mech 1 Buttons
    public static final int kShoot =                     1;
    public static final int kIntake =                    3;
    public static final int kAdvancedShoot =             2;
    public static final int kClimberHook =               4;
    public static final int kClimberClimb =              5;
    //Game Mech 2 Buttons
    public static final int kDriveToClimb =              1; 

    public static final double kDeadband = 0.075;

    // testMode buttons
    public static final int RIGHTSTICKVERT        = 5;  // driver
    public static final int TEST_INTAKE_ROLLER    = 1;  // A
    public static final int TEST_INTAKE_JOINT     = 2;  // A
    public static final int TEST_CLIMB            = 3;  // A
    public static final int TEST_SHOOT            = 4;  // A
    public static final int TEST_CONVEYOR         = 5;  // A
    public static final int SOFTLIMIT_DISABLE     = 6;  // A
    public static final int SOFTLIMIT_ENABLE      = 7;  // A
    public static final int RESETENCODER          = 11; // A
    public static final int TEST_LOW_CONTROL      = 9;  // A
    public static final int TEST_HIGH_CONTROL     = 10; // A
    public static final int TEST_CLMODE           = 2;  // B
    public static final int TEST_INTAKE_LEFT_JOINT= 3;  // B
    public static final int TEST_INTAKE_RIGHT_JOINT=4;  // B
  }

  public static class CamConstants {
    public static final String camera_name = "Camera_Module_v2";
    public static final String camera_name2 = "IDK";
      //public static final double camera_Height_Meters = Units.inchesToMeters(7.);
      //public static final double target_Height_Meters = Units.inchesToMeters(78.);

      public static final double camera_Height_Meters = Units.inchesToMeters(7);
      public static final double target_Height_Meters = Units.inchesToMeters(12);
      public static final double camera_Pitch_Radians = Units.degreesToRadians(-4.727443456);
      //public static final double tag_Follow_P = 1.75;
      //public static final double tag_Follow_D = 0.5;
      public static final double drive_Range_Meters = 1;

      public static final Transform3d robot_to_camera = new Transform3d(
                                                        0,
                                                        0,
                                                        0,
                                                        new Rotation3d(0.0,0.0,0.0)); //TODO: Add the real numbers
      public static final Transform3d robot_to_camera2 = new Transform3d(
                                                        0,
                                                        0,
                                                        0,
                                                        new Rotation3d(0.0,0.0,0.0));

      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class RobotConstants {
    public static final double xcg = Units.inchesToMeters(0);  // from the geometric centroid
    public static final double kNominalVoltage = 12.0;
    public static final double kPeriod = TimedRobot.kDefaultPeriod;
    public static final double robotLength = Units.inchesToMeters(33.75); //2026 inches  With Bumpers
    public static final double robotWidth = Units.inchesToMeters(33.75) ; //2026 inches  With Bumpers
  }

  public static final class PathPlannerConstants {
    public static final boolean isCompetition = false;
    public static final PathConstraints pathConstraints = new PathConstraints(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond,
                          DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond,
                          DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond,
                          DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    /*public static final PathConstraints pathConstraints = new PathConstraints(2,
                          4,
                          3,
                          1);*/
  }

  public static class FieldConstants{
    //Center of the Hexagon, Top of the funnel 
    public static final Pose3d blueHubFieldPose = new Pose3d(Units.inchesToMeters(157.79+23.51),
      Units.inchesToMeters(158.32), Units.inchesToMeters(72.00), new Rotation3d(new Rotation2d(180))); //The angle the scoring side faces
    public static final Pose3d redHubFieldPose = new Pose3d(Units.inchesToMeters(444.80+23.51), 
      Units.inchesToMeters(158.32), Units.inchesToMeters(72.00), new Rotation3d(new Rotation2d(0)));  //The angle the scoring side faces
    public static final VPose2d VPose2dBlueHub = new VPose2d(blueHubFieldPose.toPose2d());
    public static final VPose2d VPose2dRedHub = new VPose2d(redHubFieldPose.toPose2d());

    public static final Pose2d blueTowerLClimb = new Pose2d(Units.inchesToMeters(60.55),    
      Units.inchesToMeters(129.86), new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d blueTowerRClimb = new Pose2d(Units.inchesToMeters(60.55),
      Units.inchesToMeters(163.86), new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d redTowerLClimb = new Pose2d(Units.inchesToMeters(589.57),
      Units.inchesToMeters(118.78), new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d redTowerRClimb = new Pose2d(Units.inchesToMeters(589.57),
      Units.inchesToMeters(152.78), new Rotation2d(Units.degreesToRadians(180)));
    public static final String noPath = "None";
    public static final String OutpostPath = "Drive To Outpost";
    public static final String LeftClimbPath = "Drive Left Climb";
    public static final String RightClimbPath = "Drive Right Climb"; 
    public static final Pose2d NeutralZoneLeft = new Pose2d(6.5, 5.5, new Rotation2d(0)); //Meters (NOT MEASURED FROM SOMETHING)
    public static final Pose2d NeutralZoneRight = new Pose2d(6.5, 2.5, new Rotation2d(0)); //Meters (NOT MEASURED FROM SOMETHING)


  }

}
