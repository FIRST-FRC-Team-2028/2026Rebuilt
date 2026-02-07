// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VPose2d;
import frc.robot.Constants.PathPlannerConstants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetInRange extends Command {
  private final Drivetrain drivesubsystem;
  Optional<Alliance> alliance;
  double range, dist, theta, endVelocity;
  VPose2d whereIam, diff, whereToGo;
  Pose2d whereToDrive;
  /** Creates a new GetInRange. */
  public GetInRange(Drivetrain drivesubsystem, Optional<Alliance> alliance, double range, double endVelocity) {
    this.drivesubsystem=drivesubsystem;
    this.alliance = alliance;
    this.range = range;
    this.endVelocity = endVelocity;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    whereIam = new VPose2d(drivesubsystem.getPoseEstimatorPose());
    diff = drivesubsystem.getVecToHub(alliance);
    dist = diff.norm() - range;
    whereToGo = whereIam.plus(diff.unit().scalarProd(dist));
    theta = Units.radiansToDegrees(Math.atan(diff.Y()/diff.X()));
    if (alliance.get()==Alliance.Red)theta += 180;
    if(dist<=0){ whereToGo = whereIam;}

    SmartDashboard.putNumber("dist", dist);
    SmartDashboard.putNumber("TargetX", whereToGo.X());
    SmartDashboard.putNumber("TargetY", whereToGo.Y());
    SmartDashboard.putNumber("TargetTheta", theta);
    whereToDrive = new Pose2d(whereToGo.X(), whereToGo.Y(), new Rotation2d(Units.degreesToRadians(theta)));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AutoBuilder.pathfindToPose(whereToDrive, PathPlannerConstants.pathConstraintsTest, endVelocity);
    //drivesubsystem.goTorange(alliance, range, drivesubsystem.getPoseEstimatorPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
