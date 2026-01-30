// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  private final SparkMax leftShooter, centerShooter, rightShooter, conveyor, feed;
  private final SparkMaxConfig left_Config, center_Config, right_Config, feed_Config, conveyor_Config;
  private final RelativeEncoder center_Encoder, conveyor_Encoder, feed_Encoder;
  private final SparkClosedLoopController center_ClosedLoopController, conveyor_ClosedLoopController, feed_ClosedLoopController;
  /** Creates a new Shooter Subsytem. */
  public Shooter() {
    leftShooter = new SparkMax(CANIDS.leftShooter, MotorType.kBrushless);
    centerShooter = new SparkMax(CANIDS.centerShooter, MotorType.kBrushless);
    rightShooter = new SparkMax(CANIDS.rightShooter, MotorType.kBrushless);
    conveyor = new SparkMax(CANIDS.conveyor, MotorType.kBrushless);
    feed = new SparkMax(CANIDS.feed, MotorType.kBrushless);

    left_Config = new SparkMaxConfig();
    center_Config = new SparkMaxConfig();
    right_Config = new SparkMaxConfig();
    conveyor_Config = new SparkMaxConfig();
    feed_Config = new SparkMaxConfig();

    center_Encoder = centerShooter.getEncoder();
    conveyor_Encoder = conveyor.getEncoder();
    feed_Encoder = feed.getEncoder();

    center_ClosedLoopController = centerShooter.getClosedLoopController();
    conveyor_ClosedLoopController = conveyor.getClosedLoopController();
    feed_ClosedLoopController = feed.getClosedLoopController();


    center_Config
      .idleMode(IdleMode.kCoast);
    center_Config.encoder
      .velocityConversionFactor(ShooterConstants.shooterVelocityConversionFactor);
    center_Config.closedLoop
      .pid(ShooterConstants.shooterP, ShooterConstants.shooterI, ShooterConstants.shooterD);
    
    left_Config.follow(CANIDS.centerShooter);
    right_Config.follow(CANIDS.centerShooter);

    conveyor_Config
      .idleMode(IdleMode.kCoast);
    conveyor_Config.encoder
      .velocityConversionFactor(ShooterConstants.conveyorVelocityConversionFactor);
    conveyor_Config.closedLoop
      .pid(ShooterConstants.conveyorP, ShooterConstants.conveyorI, ShooterConstants.conveyorD);

    feed_Config
      .idleMode(IdleMode.kCoast);
    feed_Config.encoder
      .velocityConversionFactor(ShooterConstants.feedVelocityConversionFactor);
    feed_Config.closedLoop
      .pid(ShooterConstants.feedP, ShooterConstants.feedI, ShooterConstants.feedD);

    leftShooter.configure(left_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    centerShooter.configure(center_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightShooter.configure(right_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    conveyor.configure(conveyor_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feed.configure(feed_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }


  
 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed (RPM)", center_Encoder.getVelocity());
    
  }

    /**Sets the speed for the shooter using PID controller on velocity control type
   * @param Speed in RPM
   */
  public void setShooterSpeed(double Speed){
    center_ClosedLoopController.setSetpoint(Speed, ControlType.kVelocity);
  }
  /** Returns the Velocity in RPM */
  public double getShooterVelocity(){
    return center_Encoder.getVelocity();
  }
    /**Sets the speed for the conveyor using PID controller on velocity control type
   * @param Speed in RPM
   */
  public void setConveyorSpeed(double Speed){
    conveyor_ClosedLoopController.setSetpoint(Speed, ControlType.kVelocity);
  }
  /**Sets the speed for the feed using PID controller on velocity control type
   * @param Speed in RPM
   */
  public void setFeedSpeed(double Speed){
    feed_ClosedLoopController.setSetpoint(Speed, ControlType.kVelocity);
  }


}
