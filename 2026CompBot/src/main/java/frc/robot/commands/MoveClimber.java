// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.util.Elastic;
import frc.robot.util.HubTracker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveClimber extends Command {
    private final Climber climber;
    private final Intake intake;
    double deadband = 5, climblevel, loopcounter =0; 
    boolean intakeout = false, endGame = false, startedClimb = false, auto;
  /** Climb to desired "height" 
   * @param position  ie, travel, engage, level1 or level2
  */
  public MoveClimber(Climber climber, Intake intake, boolean auto) {
    this.intake = intake;
    this.climber = climber;
    this.auto = auto;
    addRequirements(climber, intake);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climblevel = climber.getClimbLevel();
    intakeout = intake.getIntakeOut();
    if (HubTracker.getCurrentShift().isPresent()) endGame = HubTracker.getCurrentShift().get().equals(HubTracker.Shift.ENDGAME);
    loopcounter = 0;
    if (!auto){
      if (endGame && !intakeout) switchClimberPose();
      if (!endGame){ climber.setClimberPosition(ClimberConstants.travelPosition);startedClimb=false;}
    } else{ 
      climblevel = ClimberConstants.climbLevel1; 
      if (!intakeout) switchClimberPose();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeout = intake.getIntakeOut();
    if (!auto){
      if (endGame && intakeout && loopcounter==0){
        intake.setJointPosition(IntakeConstants.JointUpPosition);
        loopcounter = 1;
      }
      if (!startedClimb && !intakeout && endGame) switchClimberPose();

      if (climblevel == ClimberConstants.climbLevel2 && !intakeout
          && climber.getClimberPosition()>ClimberConstants.sendOutIntakePose-deadband && climber.getClimberPosition()<ClimberConstants.sendOutIntakePose+deadband){
          intake.setJointPosition(IntakeConstants.JointClimbPosition);
      } 
    } else{
      if (intakeout && loopcounter==0){
        intake.setJointPosition(IntakeConstants.JointUpPosition);
        loopcounter = 1;
      }
      if (!startedClimb && !intakeout) switchClimberPose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!endGame && !auto) return true;
    return false;
  }
  /** If the climber pose is at travel:Go to Climb Level. If the climber pose isn't at travel: Go to travel */
  private void switchClimberPose(){
    if (climber.getClimberPosition()<ClimberConstants.travelPosition+deadband&&climber.getClimberPosition()>ClimberConstants.travelPosition-deadband){
    climber.setClimberPosition(climblevel); 
    startedClimb = true;
    }else {climber.setClimberPosition(ClimberConstants.travelPosition); startedClimb=false;}
  }
}
