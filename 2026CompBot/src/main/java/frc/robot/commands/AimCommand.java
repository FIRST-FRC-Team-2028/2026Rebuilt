// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VPose2d;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimCommand extends Command {
  private final Drivetrain drive;
  private Optional<Alliance> alliance;
  private PIDController controller;
  double targetTheta, kP=1.9/10., kI, kD, theta, speed, deadband = 1., donecount = 0;
  VPose2d diff;
  /** Creates a new AimCommand. */
  public AimCommand(Drivetrain drive, Optional<Alliance> alliance) {
    this.drive = drive;
    this.alliance = alliance;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    theta = drive.getPoseEstimatorPose().getRotation().getDegrees();
    diff = drive.getVecToHub(alliance);
    targetTheta =  Units.radiansToDegrees(Math.atan(diff.Y()/diff.X()));
    controller = new PIDController(kP, kI, kD);
    donecount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    theta = drive.getPoseEstimatorPose().getRotation().getDegrees();
    speed = controller.calculate(theta, targetTheta);
    drive.drive(new ChassisSpeeds(0, 0, speed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DoneAim");
    drive.drive(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(targetTheta-theta) < deadband){
      donecount++;
    }
    return donecount>25;
  }
}
