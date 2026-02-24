// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.Set;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AdvancedShoot;
import frc.robot.commands.AgitateIntake;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.AprilTags;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PixyCamReader;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final Drivetrain driveSubsystem;
  private final AprilTags aprilSubsystem;
  private final Shooter shootingSubsystem;
  private final Intake intakeSubsystem;
  private final Climber climberSubsystem;
  private final PixyCamReader pixy;
  public static Optional<Alliance> alliance = DriverStation.getAlliance();
  private  SendableChooser<Command> autoChooser = null;
  Pose2d mechTargetPose;
  String mechPathName;
  boolean compButtons = false; //Temporary until we switch button boards

  // Joysticks
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick mechJoytick1 = new Joystick(OIConstants.kMechControllerPort);
    private final Joystick mechJoytick2 = new Joystick(OIConstants.kMechControllerPort2);


  public RobotContainer() {
    if (Constants.CAMERA_AVAILABLE){
      aprilSubsystem = new AprilTags();
    } else aprilSubsystem = null;
    if (Constants.SHOOTER_AVAILABLE){
      shootingSubsystem = new Shooter();
    } else shootingSubsystem = null;
    if (Constants.INTAKE_AVAILABLE){
      intakeSubsystem = new Intake();
    } else intakeSubsystem = null;
    if (Constants.CLIMBER_AVAILABLE){
      climberSubsystem = new Climber(alliance);
    } else climberSubsystem = null;
    if (Constants.PIXYCAM_AVAILABLE){
      pixy = new PixyCamReader();
    } else pixy = null;
    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem = new Drivetrain(aprilSubsystem);
      driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driverJoytick));
    } else driveSubsystem = null;

    if (Constants.DRIVE_AVAILABLE){
       NamedCommands.registerCommand("Drive To Shoot", Commands.defer(
          ()->driveSubsystem.pathfindToPose(driveSubsystem.getTorange(alliance, 2.25, 1.5), 0), Set.of(driveSubsystem)).alongWith(new InstantCommand(()->System.out.println("Working"))));
      NamedCommands.registerCommand("PathfindToClimbLeftPath", driveSubsystem.pathfindToPath("Drive Climb Left"));
      NamedCommands.registerCommand("PathfindToClimbRightPath", driveSubsystem.pathfindToPath("Drive Climb Right"));
      if (Constants.INTAKE_AVAILABLE){
        NamedCommands.registerCommand("Intake Out", new InstantCommand(()->intakeSubsystem.setJointPosition(IntakeConstants.JointPickupPosition)));
        NamedCommands.registerCommand("Run Intake", intakeSubsystem.runIntake(IntakeConstants.IntakeSpeed));
        if (Constants.SHOOTER_AVAILABLE){
          NamedCommands.registerCommand("Shoot Sequence", new Shoot(shootingSubsystem, ShooterConstants.OptimalShootSpeed).alongWith(new AgitateIntake(intakeSubsystem)));
          //NamedCommands.registerCommand("Advanced Shoot Sequence", new AdvancedShoot(shootingSubsystem, driveSubsystem.getVecToHub(alliance).norm()).alongWith(new AgitateIntake(intakeSubsystem)));
        }
      }
      if(Constants.CLIMBER_AVAILABLE){
        NamedCommands.registerCommand("Climb", null);
      }
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> PathPlannerConstants.isCompetition
          ? stream.filter(auto -> auto.getName().startsWith("Comp"))
          : stream);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    configureBindings();
  }

  @SuppressWarnings("unused")
  public void configureBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    if (Constants.DRIVE_AVAILABLE){
      new JoystickButton(driverJoytick, OIConstants.kResetGyro)
        .onTrue(new InstantCommand(()->driveSubsystem.resetGyro()));
      new JoystickButton(driverJoytick, OIConstants.kDriveToShootRange)
        .whileTrue(Commands.defer(()->driveSubsystem.pathfindToPose(
            driveSubsystem.getTorange(alliance, ShooterConstants.OptimalRange, ShooterConstants.MinRange), 0), Set.of(driveSubsystem)));
      new JoystickButton(driverJoytick, OIConstants.kDriveToMechPose)
      .whileTrue(Commands.defer(()->driveSubsystem.pathfindToPoseOrPath(mechTargetPose, 0, mechPathName), Set.of(driveSubsystem)));
      //Game Mech Set Target Buttons
      new JoystickButton(mechJoytick1, 90) //Outpost
        .onTrue(new InstantCommand(()->mechPathName=FieldConstants.OutpostPath));
      new JoystickButton(mechJoytick1, 90) //Climb
        .onTrue(new InstantCommand(()->mechPathName=climberSubsystem.getWhereToClimb()));
      new JoystickButton(mechJoytick1, 90) //Neutral Zone Left
        .onTrue(new InstantCommand(()->mechPathName=FieldConstants.noPath).alongWith(new InstantCommand(()->mechTargetPose=FieldConstants.NeutralZoneLeft)));
      new JoystickButton(mechJoytick1, 90) //Neutral Zone Right
        .onTrue(new InstantCommand(()->mechPathName=FieldConstants.noPath).alongWith(new InstantCommand(()->mechTargetPose=FieldConstants.NeutralZoneRight)));
        
      if (Constants.CLIMBER_AVAILABLE){
        new JoystickButton(mechJoytick2, OIConstants.kDriveToClimb) //Drive to the selected Climb Location
          .whileTrue(
            Commands.defer(()->driveSubsystem.pathfindToPoseOrPath(new Pose2d(), 0, climberSubsystem.getWhereToClimb()), Set.of(driveSubsystem))
        );}
    }

    if (Constants.INTAKE_AVAILABLE && Constants.SHOOTER_AVAILABLE){
      /*new JoystickButton(mechJoytick1, OIConstants.kShoot)
        .whileTrue(new Shoot(shootingSubsystem)
        .alongWith(new AgitateIntake(intakeSubsystem)));

      new JoystickButton(mechJoytick1, OIConstants.kAdvancedShoot)
        .whileTrue(new AdvancedShoot(shootingSubsystem, driveSubsystem.getVecToHub(alliance).norm())
        .alongWith(new AgitateIntake(intakeSubsystem)));*/

      /* new JoystickButton(mechJoytick1, OIConstants.kIntake)
        .onTrue(intakeSubsystem.runIntake(IntakeConstants.IntakeSpeed))
        .onFalse(intakeSubsystem.stopIntake()); */

    }
    
    if (Constants.CLIMBER_AVAILABLE){
      /*new JoystickButton(mechJoytick1, OIConstants.kClimberHook)
        .onTrue(climberSubsystem.MoveClimber(ClimberConstants.hookPosition));

      new JoystickButton(mechJoytick1, OIConstants.kClimberClimb)
        .onTrue(climberSubsystem.MoveClimber(ClimberConstants.climbPosition));*/
      new JoystickButton(mechJoytick1, 1)
        .onTrue(new InstantCommand(()-> climberSubsystem.setClimberSpeed(.3)))
        .onFalse(new InstantCommand(()-> climberSubsystem.setClimberSpeed(0)));
      new JoystickButton(mechJoytick1, 2)
        .onTrue(new InstantCommand(()-> climberSubsystem.setClimberSpeed(-.3)))
        .onFalse(new InstantCommand(()-> climberSubsystem.setClimberSpeed(0)));
  
    }
    if (Constants.SHOOTER_AVAILABLE){
      new JoystickButton(mechJoytick1, 1) //Set Shooter MAX speed
        .onTrue(new InstantCommand(()->shootingSubsystem.setShooterSpeed(5676)))
        .onFalse(new InstantCommand(()->shootingSubsystem.stopShooting()));
      new JoystickButton(mechJoytick1, 2) //Set Shooter MAX speed
        .onTrue(new InstantCommand(()->shootingSubsystem.setShooterSpeed(ShooterConstants.OptimalShootSpeed)))
        .onFalse(new InstantCommand(()->shootingSubsystem.stopShooting()));
      //new JoystickButton(mechJoytick1, 2)
      //  .onTrue(new AdvancedShoot(shootingSubsystem, 3));
      new JoystickButton(mechJoytick1, 3)
        .whileTrue(new AdvancedShoot(shootingSubsystem, Units.feetToMeters(15)));
            new JoystickButton(mechJoytick1, 4)
        .whileTrue(new AdvancedShoot(shootingSubsystem, Units.feetToMeters(10)));
    }
    if (Constants.INTAKE_AVAILABLE){
      new JoystickButton(mechJoytick1, 5)
        .onTrue(new InstantCommand(()->intakeSubsystem.setJointPosition(IntakeConstants.JointPickupPosition)));
      new JoystickButton(mechJoytick1, 6)
        .onTrue(new InstantCommand(()->intakeSubsystem.setJointPosition(IntakeConstants.JointUpPosition)));
      new JoystickButton(mechJoytick1, 7)
        .onTrue(new InstantCommand(()->intakeSubsystem.rollers(-.3)))
        .onFalse(new InstantCommand(()->intakeSubsystem.rollers(0)));
    }

    if (compButtons){//Temporary until we switch button boards
      if (Constants.DRIVE_AVAILABLE){
        new JoystickButton(driverJoytick, OIConstants.kResetGyro)
          .onTrue(new InstantCommand(()->driveSubsystem.resetGyro()));
        new JoystickButton(driverJoytick, OIConstants.kDriveToShootRange)
          .whileTrue(Commands.defer(()->driveSubsystem.pathfindToPose(
            driveSubsystem.getTorange(alliance, ShooterConstants.OptimalRange, ShooterConstants.MinRange), 0), Set.of(driveSubsystem)));
        new JoystickButton(driverJoytick, OIConstants.kDriveToMechPose)
          .whileTrue(Commands.defer(()->driveSubsystem.pathfindToPoseOrPath(mechTargetPose, 0, mechPathName), Set.of(driveSubsystem)));
      //Game Mech Set Target Buttons
       new JoystickButton(mechJoytick1, 1) //Neutral Zone Left
          .onTrue(new InstantCommand(()->mechPathName=FieldConstants.noPath).alongWith(new InstantCommand(()->mechTargetPose=FieldConstants.NeutralZoneLeft)));
        new JoystickButton(mechJoytick1, 2) //Neutral Zone Right
          .onTrue(new InstantCommand(()->mechPathName=FieldConstants.noPath).alongWith(new InstantCommand(()->mechTargetPose=FieldConstants.NeutralZoneRight)));
        new JoystickButton(mechJoytick1, 3) //Neutral Zone Left
          .onTrue(new InstantCommand(()->mechPathName=FieldConstants.noPath).alongWith(new InstantCommand(()->mechTargetPose=FieldConstants.AllianceZoneLeft)));
        new JoystickButton(mechJoytick1, 4) //Neutral Zone Right
          .onTrue(new InstantCommand(()->mechPathName=FieldConstants.noPath).alongWith(new InstantCommand(()->mechTargetPose=FieldConstants.AllianeZoneRight)));
        if (Constants.CLIMBER_AVAILABLE) new JoystickButton(mechJoytick1, 5) //Climb
          .onTrue(new InstantCommand(()->mechPathName=climberSubsystem.getWhereToClimb()));
        new JoystickButton(mechJoytick1, 6) //Outpost
          .onTrue(new InstantCommand(()->mechPathName=FieldConstants.OutpostPath));
       }
      if (Constants.INTAKE_AVAILABLE){
        new JoystickButton(mechJoytick2, 1)
          .onTrue(intakeSubsystem.toggleJointPosition());
        new JoystickButton(mechJoytick2, 6)
          .onTrue(intakeSubsystem.toggleRunIntake(IntakeConstants.IntakeSpeed));
      }
      if (Constants.CLIMBER_AVAILABLE){
        new JoystickButton(mechJoytick2, 7)
          .onTrue(new MoveClimber(climberSubsystem));
      }
      if (Constants.SHOOTER_AVAILABLE){
        new JoystickButton(mechJoytick2, 2)
          .whileTrue(new Shoot(shootingSubsystem, ShooterConstants.OptimalShootSpeed).alongWith(new AgitateIntake(intakeSubsystem)));
        new JoystickButton(mechJoytick2, 8)
          .onTrue(new AdvancedShoot(shootingSubsystem, driveSubsystem.getVecToHub(alliance).norm()).alongWith(new AgitateIntake(intakeSubsystem)));
        new JoystickButton(mechJoytick2, 3)
          .onTrue(new InstantCommand(()-> shootingSubsystem.incrementShootSpeed(50)));
        new JoystickButton(mechJoytick2, 9)
          .onTrue(new InstantCommand(()-> shootingSubsystem.incrementShootSpeed(50)));
      }
    
    }
  }

 /*  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  } */
  public Drivetrain getDrivetrain(){
    return driveSubsystem;
  }
  public Intake getIntake(){
    return intakeSubsystem;
  }
  public Shooter getShoot(){
    return shootingSubsystem;
  }
  public Climber getClimber(){
    return climberSubsystem;
  }
  public PixyCamReader getPixyCam(){
    return pixy;
  }
  public AprilTags getAprilTags(){
    return aprilSubsystem;
  }

  public Joystick getDriverJoystick(){
    return driverJoytick;
  }
  public Joystick getMech1Joystick(){
    return mechJoytick1;
  }
  public Joystick getMech2Joystick(){
    return mechJoytick2;
  }
  public Optional<Alliance> getAlliance(){
    return alliance;
  }
  /** disable drive defaultCommand  - for testing*/
  public void turnOffDrive(){
    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem.removeDefaultCommand();
    }
  }
  /** restore drive defaultCommand  - from testing*/
  public void turnOnDrive(){
    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driverJoytick));
    }
  }
}
