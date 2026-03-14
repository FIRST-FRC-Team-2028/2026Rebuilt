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
import com.pathplanner.lib.events.EventTrigger;

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
import frc.robot.commands.DriveToRangeAndShoot;
import frc.robot.commands.MoveClimber;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.AprilTags;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final Drivetrain driveSubsystem;
  private final AprilTags aprilSubsystem;
  private final Shooter shootingSubsystem;
  private final Intake intakeSubsystem;
  private final Climber climberSubsystem;
  public static Optional<Alliance> alliance = DriverStation.getAlliance();
  private  SendableChooser<Command> autoChooser = null;
  Pose2d mechTargetPose;
  String mechPathName;

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

    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem = new Drivetrain(aprilSubsystem);
      driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driverJoytick));
    } else driveSubsystem = null;

    if (Constants.DRIVE_AVAILABLE){
      NamedCommands.registerCommand("PathfindToClimbLeftPath", driveSubsystem.pathfindToPath("Drive Left Climb"));
      NamedCommands.registerCommand("PathfindToClimbRightPath", driveSubsystem.pathfindToPath("Drive Right Path"));
      NamedCommands.registerCommand("PathfindToLeftPathfindToCenter", driveSubsystem.pathfindToPath("Left Pathfind To Center"));
      NamedCommands.registerCommand("PathfindToRightPathfindToCenter", driveSubsystem.pathfindToPath("Right Pathfind To Center"));
      NamedCommands.registerCommand("PathfindToDepot", driveSubsystem.pathfindToPath("Pathfind To Deopt"));
      NamedCommands.registerCommand("PathfindToOutpost", driveSubsystem.pathfindToPath("Drive To Outpost"));

      if (Constants.INTAKE_AVAILABLE){
        new EventTrigger("Intake Out");
        new EventTrigger("Intake In");
        NamedCommands.registerCommand("Intake Out", new InstantCommand(()->intakeSubsystem.setJointPosition(IntakeConstants.JointPickupPosition)));
        NamedCommands.registerCommand("Intake In", new InstantCommand(()->intakeSubsystem.setJointPosition(IntakeConstants.JointUpPosition)));
        NamedCommands.registerCommand("Run Intake", intakeSubsystem.runIntake(IntakeConstants.IntakeSpeed));
        NamedCommands.registerCommand("Stop Intake", intakeSubsystem.stopIntake());
        if (Constants.SHOOTER_AVAILABLE){
          NamedCommands.registerCommand("Shoot Sequence", new Shoot(shootingSubsystem, ShooterConstants.OptimalShootSpeed).alongWith(new AgitateIntake(intakeSubsystem)));
          NamedCommands.registerCommand("Drive To Shoot", new DriveToRangeAndShoot(driveSubsystem, shootingSubsystem, intakeSubsystem, alliance, true, 10));
          //NamedCommands.registerCommand("Advanced Shoot Sequence", new AdvancedShoot(shootingSubsystem, driveSubsystem.getVecToHub(alliance).norm()).alongWith(new AgitateIntake(intakeSubsystem)));
        }
      }
      if(Constants.CLIMBER_AVAILABLE){
        //NamedCommands.registerCommand("Climb", new MoveClimber(climberSubsystem, intakeSubsystem, true));
        NamedCommands.registerCommand("Climb", new InstantCommand(()->System.out.println("climb")));

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


    if (compButtons){//Temporary until we switch button boards
      if (Constants.DRIVE_AVAILABLE){
        new JoystickButton(driverJoytick, OIConstants.kResetGyro)
          .onTrue(new InstantCommand(()->driveSubsystem.resetGyro()));
        /*new JoystickButton(driverJoytick, OIConstants.kDriveToShootRange)
          .whileTrue(Commands.defer(()->driveSubsystem.pathfindToPose(
            driveSubsystem.getTorange(alliance, ShooterConstants.OptimalRange, ShooterConstants.MinRange), 0), Set.of(driveSubsystem)));*/
        if(Constants.INTAKE_AVAILABLE && Constants.SHOOTER_AVAILABLE){
          new JoystickButton(driverJoytick, OIConstants.kDriveToShootRange)
            .whileTrue(new DriveToRangeAndShoot(driveSubsystem, shootingSubsystem, intakeSubsystem, alliance, false, 4));
        }
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
          .onTrue(Commands.defer(()->intakeSubsystem.toggleJointPosition(), Set.of(intakeSubsystem)));
        new JoystickButton(mechJoytick2, 6)
          .onTrue(new InstantCommand(()->intakeSubsystem.toggleIntakeWheels(IntakeConstants.IntakeSpeed)));
      }
      if (Constants.CLIMBER_AVAILABLE){
        new JoystickButton(mechJoytick2, 7)
          .onTrue(new MoveClimber(climberSubsystem, intakeSubsystem, false));
      }
      if (Constants.SHOOTER_AVAILABLE){
        if(Constants.INTAKE_AVAILABLE){
          new JoystickButton(mechJoytick2, 2)
            .whileTrue(new Shoot(shootingSubsystem, ShooterConstants.OptimalShootSpeed)
                          .alongWith(new AgitateIntake(intakeSubsystem)));
        } else
           new JoystickButton(mechJoytick2, 2)
            .whileTrue(new Shoot(shootingSubsystem, ShooterConstants.OptimalShootSpeed));
        new JoystickButton(mechJoytick2, 4)
          .whileTrue(new Shoot(shootingSubsystem, ShooterConstants.OptimalShootSpeed));
        if(Constants.DRIVE_AVAILABLE){
          if(Constants.INTAKE_AVAILABLE){
            new JoystickButton(mechJoytick2, 8)
            .onTrue(new AdvancedShoot(shootingSubsystem, driveSubsystem.getVecToHub(alliance).norm()).alongWith(new AgitateIntake(intakeSubsystem)));
          } else
            new JoystickButton(mechJoytick2, 8)
              .onTrue(new AdvancedShoot(shootingSubsystem, driveSubsystem.getVecToHub(alliance).norm())); 
        }
        new JoystickButton(mechJoytick2, 3)
          .onTrue(new InstantCommand(()-> shootingSubsystem.incrementShootSpeed(50)));
        new JoystickButton(mechJoytick2, 9)
          .onTrue(new InstantCommand(()-> shootingSubsystem.incrementShootSpeed(-50)));
        new JoystickButton(mechJoytick2, 5)
          .onTrue(new Shoot(shootingSubsystem, 4250));
      }
    
    }
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  } 
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
