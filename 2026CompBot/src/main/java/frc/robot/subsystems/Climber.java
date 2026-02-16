// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import java.util.Optional;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.CANIDS;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final SparkMax climber, follower;
  private final SparkMaxConfig climber_Config, follower_Config;
  private final RelativeEncoder climber_Encoder;
  private final SparkClosedLoopController climber_ClosedLoopController;
  private SendableChooser<String> whereToClimb;
  Optional<Alliance> alliance;

  /** Creates a new Climber. */
  public Climber(Optional<Alliance> alliance) {
    climber = new SparkMax(CANIDS.climber, MotorType.kBrushless);
    follower = new SparkMax(CANIDS.climber_follower, MotorType.kBrushless);

    climber_Config = new SparkMaxConfig();
    follower_Config = new SparkMaxConfig();

    climber_Encoder = climber.getEncoder();
    climber_ClosedLoopController = climber.getClosedLoopController();

    climber_Config
      .closedLoopRampRate(ClimberConstants.kRampRate)
      .idleMode(IdleMode.kBrake);
    climber_Config.softLimit
      .forwardSoftLimit(ClimberConstants.forwardSoftLimit)
      .reverseSoftLimit(ClimberConstants.reverseSoftLimit)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);
    climber_Config.encoder
      .positionConversionFactor(ClimberConstants.positionConversionFactor);
    climber_Config.closedLoop
      .pid(ClimberConstants.p, ClimberConstants.i, ClimberConstants.d);

    follower_Config
      .inverted(true)
      .follow(climber);

    climber.configure(climber_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower.configure(follower_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.alliance = alliance;
    whereToClimb = new SendableChooser<String>();
      whereToClimb.setDefaultOption("Left L1", FieldConstants.LeftClimbPath);
      whereToClimb.addOption("Right L1", FieldConstants.RightClimbPath);
      //whereToClimb.addOption("Left L2", FieldConstants.redTowerLClimb);
      //whereToClimb.addOption("Right L2", FieldConstants.redTowerRClimb);
    SmartDashboard.putData("Where To CLimb", whereToClimb);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**In Degrees */
  public void setClimberPosition(double Position){
    Position = Units.degreesToRotations(Position);
    climber_ClosedLoopController.setSetpoint(Position, ControlType.kPosition);
  }
  /**Sets climber speed. Between -1 and 1 */
  public void setClimberSpeed(double speed){
    climber.set(speed);
  }
  /**Returns position in degrees*/
  public double getClimberPosition(){
    return Units.rotationsToDegrees(climber_Encoder.getPosition());
  }
  /**Returns output current in amps */
  public double getClimberCurrent(){
    return climber.getOutputCurrent();
  }
  public void switchSoftLimits(boolean enabled, boolean setPosition){
    climber_Config.softLimit.forwardSoftLimitEnabled(enabled);
    climber_Config.softLimit.reverseSoftLimitEnabled(enabled);
    climber.configure(climber_Config, ResetMode.kResetSafeParameters, null);
    if (setPosition) climber_Encoder.setPosition(ClimberConstants.travelPosition);
  }

  /**Set the position of the climber in degrees */
  public Command MoveClimber(double position){
    return new InstantCommand(()->setClimberPosition(position));
  }

  public String getWhereToClimb(){
    return whereToClimb.getSelected();
  }

}
