// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final SparkMax climber, follower;
  private final SparkMaxConfig climber_Config, follower_Config;
  private final RelativeEncoder climber_Encoder;
  private final SparkClosedLoopController climber_ClosedLoopController;
  /** Creates a new Climber. */
  public Climber() {
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

}
