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
  private final SparkMax wheels;
  private final SparkMax followerWheels;
  private final SparkMaxConfig wheels_Config;
  private final SparkMaxConfig followerWheels_Config;
  private final RelativeEncoder wheels_Encoder;
  private final SparkClosedLoopController wheels_ClosedLoopController;
  double speed = 200.0; //3400 target speed
  double incrementAmount = 200.0;
  /** Creates a new Shooter. */
  public Shooter() {
    wheels = new SparkMax(CANIDS.wheels, MotorType.kBrushless);
    followerWheels = new SparkMax(CANIDS.followerWheels, MotorType.kBrushless);
    wheels_Config = new SparkMaxConfig();
    followerWheels_Config = new SparkMaxConfig();

    wheels_Encoder = wheels.getEncoder();
    wheels_ClosedLoopController = wheels.getClosedLoopController();

    wheels_Config
    .idleMode(IdleMode.kCoast);
    wheels_Config.encoder
    .velocityConversionFactor(ShooterConstants.velocityConversionFactor);
    wheels_Config.closedLoop
    .pid(ShooterConstants.p, ShooterConstants.i, ShooterConstants.d);
    
    followerWheels_Config.follow(CANIDS.wheels);

    wheels.configure(wheels_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerWheels.configure(followerWheels_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**Sets the speed for the wheels using PID controller on velocity control type
   * @param Speed in RPM
   */
  public void setSpeed(double Speed){
    wheels_ClosedLoopController.setSetpoint(Speed, ControlType.kVelocity);
  }
  public void dashboardSetSpeed(){
    wheels_ClosedLoopController.setSetpoint(SmartDashboard.getNumber("Command Speed", speed), ControlType.kVelocity);
  }
 @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Speed (RPM)", wheels_Encoder.getVelocity());
    SmartDashboard.putNumber("Shooter Command Speed", speed);
    
  }
}
