// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToRangeAndShoot extends SequentialCommandGroup {
  /** Creates a new DriveToRangeAndShoot. */
  public DriveToRangeAndShoot(Drivetrain drive, Shooter shooter, Intake intake, Optional<Alliance> alliance) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
                Commands.parallel(
                  drive.pathfindToPose(drive.getTorange(alliance, ShooterConstants.OptimalRange, ShooterConstants.MinRange), 0),
                  new WaitUntilCommand(()->drive.distToGo()<1)  //Waits until 1 meter away from the target to start spinning the shooter wheels
                  .andThen(new InstantCommand(()->shooter.setShooterSpeed(ShooterConstants.shooterShootSpeed)))
                ),
                Commands.race(
                  new WaitCommand(5), //Gives 5 seconds to shoot (Estimate)
                  Commands.parallel(
                    new Shoot(shooter),
                    new AgitateIntake(intake)
                  )
                )


    );  

  }


}
