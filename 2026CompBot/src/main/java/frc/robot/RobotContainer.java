// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AdvancedShoot;
import frc.robot.commands.AgitateIntake;
import frc.robot.commands.DriveCommand;
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
      climberSubsystem = new Climber();
    } else climberSubsystem = null;
    if (Constants.DRIVE_AVAILABLE){
      driveSubsystem = new Drivetrain(aprilSubsystem);
      driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem, driverJoytick));
    } else driveSubsystem = null;

    configureBindings();
  }

  @SuppressWarnings("unused")
  private void configureBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    if (Constants.DRIVE_AVAILABLE){
      new JoystickButton(driverJoytick, OIConstants.kResetGyro)
        .onTrue(new InstantCommand(()->driveSubsystem.resetGyro()));
      new JoystickButton(driverJoytick, 2)
        .onTrue(new InstantCommand(()->driveSubsystem.goTorangeTEST(alliance, 1)));
    }

    if (Constants.INTAKE_AVAILABLE && Constants.SHOOTER_AVAILABLE){
      new JoystickButton(mechJoytick1, OIConstants.kShoot)
        .whileTrue(new Shoot(shootingSubsystem)
        .alongWith(new AgitateIntake(intakeSubsystem)));

      new JoystickButton(mechJoytick1, OIConstants.kAdvancedShoot)
        .whileTrue(new AdvancedShoot(shootingSubsystem, driveSubsystem.getVecToHub(alliance).norm())
        .alongWith(new AgitateIntake(intakeSubsystem)));

      new JoystickButton(mechJoytick1, OIConstants.kIntake)
        .onTrue(intakeSubsystem.runIntake(IntakeConstants.IntakeSpeed))
        .onFalse(intakeSubsystem.stopIntake());

    }
    
    if (Constants.CLIMBER_AVAILABLE){
      new JoystickButton(mechJoytick1, OIConstants.kClimberHook)
        .onTrue(climberSubsystem.MoveClimber(ClimberConstants.hookPosition));

      new JoystickButton(mechJoytick1, OIConstants.kClimberClimb)
        .onTrue(climberSubsystem.MoveClimber(ClimberConstants.climbPosition));
    }

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
  

}
