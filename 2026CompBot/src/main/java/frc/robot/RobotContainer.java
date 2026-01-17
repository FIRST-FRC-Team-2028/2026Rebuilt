// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.AprilTags;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private final Drivetrain driveSubsystem;
  private final AprilTags aprilSubsystem;
  private final Shooter shootingSubsystem;

  // Joysticks
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick mechJoytick1 = new Joystick(OIConstants.kMechControllerPort);
    private final Joystick mechJoytick2 = new Joystick(OIConstants.kMechControllerPort2);


  public RobotContainer() {
    if(Constants.DRIVE_AVAILABLE){
    driveSubsystem = new Drivetrain();
    driveSubsystem.setDefaultCommand(new DriveCommand(driveSubsystem));
    }else driveSubsystem = null;
    if(Constants.CAMERA_AVAILABLE){
    aprilSubsystem = new AprilTags();
    }else aprilSubsystem = null;
    if(Constants.SHOOTER_AVAILABLE){
      shootingSubsystem = new Shooter();
    } else shootingSubsystem = null;

    configureBindings();
  }

  private void configureBindings() {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    if (Constants.DRIVE_AVAILABLE){
      new JoystickButton(driverJoytick, OIConstants.kResetGyro)
        .onTrue(new InstantCommand(()->driveSubsystem.resetGyro()));
    }

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
