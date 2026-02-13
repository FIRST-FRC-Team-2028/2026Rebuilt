// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitateIntake extends Command {
  private final Intake intake;
  double deadband = 5; //Degrees
  double setPoint = IntakeConstants.JointPickupPosition;
  /** Shake hopper
   * <p> Presumes this Command is controlled by holding a button,
   * and ends when button is released.
   * @param intake
   */
  public AgitateIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setJointPosition(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getJointPosition()<setPoint+deadband && intake.getJointPosition()>setPoint-deadband){
      if (setPoint == IntakeConstants.JointPickupPosition){
        setPoint = IntakeConstants.JointAgitatePosition;
      } else {
        setPoint = IntakeConstants.JointPickupPosition;
      }
      intake.setJointPosition(setPoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setJointPosition(IntakeConstants.JointPickupPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
