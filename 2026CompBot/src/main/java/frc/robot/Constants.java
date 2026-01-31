// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
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
  public static final boolean CAMERA_AVAILABLE = true;
  public static final boolean SHOOTER_AVAILABLE = false;
  public static final boolean INTAKE_AVAILABLE = false;
  public static final boolean CLIMBER_AVAILABLE = false;



  public static final class ModuleConstants {
     public static final int kDriveMotorCurrentLimit = 80;
    public static final int kTurningMotorCurrentLimit = 80;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
    public static final double kDriveMotorGearRatio = 1 / 5.36;
    public static final double kTurningMotorGearRatio = 1 / 18.75;
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


    public static final double kTurningP = 2.3;//2.05; // 2025 Competition Robot   0.75
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
        public static final double kTrackWidth = Units.inchesToMeters(16.5);  //16.5     
        //
        public static final double kWheelBase = Units.inchesToMeters(22.5);   //22.5       
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
  public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * 2 * Math.PI;

  public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond /1.5;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                                     kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.8;
  public static final double kptwist = .5;

  public static final double kFineControlSpeed = .5;
  public static final double kFasterSpeed = .4;

  public static final String kAbsEncoderMagnetOffsetKey = "kAbsEncoderMagnetOffsetKey";
  public static final double kDefaultAbsEncoderOffset = 0.0;

  // Units are meters per second
  public static final double kMaxTranslationalVelocity = 4.0; // 2023 Competion Robot // max 4.5

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
    public static final double jointP = 0.;
    public static final double jointI = 0.;
    public static final double jointD = 0.;
    public static final double JointPositionConversionFactor = 1;
    public static final double RollerVelocityConversionFactor = 1;
    public static final double jointForwardSoftLimit = 0;
    public static final double jointReverseSoftLimit = 0;

    //Joint Positions
    public static final double JointPickupPosition = 90; //Degrees
    public static final double JointUpPosition = 0; //Degrees
    public static final double JointAgitatePosition = 75; //Degrees

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
    public static final double feedP = 0;
    public static final double feedI = 0;
    public static final double feedD = 0;
    public static final double feedVelocityConversionFactor = 1;

    public static final double shooterShootSpeed = 1400.; //RPM
    public static final double feedShootSpeed = 200.; //RPM
    public static final double conveyorShootSpeed = 300.; //RPM

    //Shooter Math Constants 
    public static final double gravity = 9.81; //m/s^2
    public static final double h_shooter = 15; //in ESTIMATE TODO set real height
    public static final double delta_H = Units.inchesToMeters(72)-Units.inchesToMeters(h_shooter);
    public static final double shooter_angle_deg = 0; //TODO set real angle
    public static final double wheel_diameter_meter = Units.inchesToMeters(0); //TODO set wheel diameter 
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
    public static final double climbPosition = 0;

  }

  public static class CANIDS {
    //Intake
    public static final int roller = 1000;
    public static final int joint = 1001;
    //Shooter
    public static final int leftShooter = 50;
    public static final int centerShooter = 51;
    public static final int rightShooter = 52;
    public static final int conveyor = 60;
    public static final int feed = 61;
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
    public static final int kDriverRobotOrientedButton = 6;
    //Game Mech 1 Buttons
    public static final int kShoot =                     1;
    public static final int kIntake =                    3;
    public static final int kAdvancedShoot =             2;
    public static final int kStopShooter =               4;


    public static final double kDeadband = 0.075;

    
  }

  public static class CamConstants {
    public static final String camera_name = "Camera_Module_v2";
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
                                                        new Rotation3d(0.0,0.0,0.0));

      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class RobotConstants {
    public static final double xcg = Units.inchesToMeters(0);  // from the geometric centroid
    public static final double kNominalVoltage = 12.0;
    public static final double kPeriod = TimedRobot.kDefaultPeriod;
    public static final double robotLength = Units.inchesToMeters(34.5); //inches
    public static final double robotWidth = Units.inchesToMeters(29.25) ; //inches
    public static final double handlerThickness = Units.inchesToMeters(6.); //inches
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
      Units.inchesToMeters(158.32), Units.inchesToMeters(72.00), null);
    public static final Pose3d redHubFieldPose = new Pose3d(Units.inchesToMeters(444.80+23.51), 
      Units.inchesToMeters(158.32), Units.inchesToMeters(72.00), null);
  }

}
