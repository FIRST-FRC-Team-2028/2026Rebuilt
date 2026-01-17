// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.CarouselConstants;

public class Carousel extends SubsystemBase {
  private final SparkFlex carousel;
  private final SparkFlexConfig carousel_Config;
  private final RelativeEncoder carousel_Encoder;
  private final SparkClosedLoopController carousel_ClosedLoopController;
  double speed = 600; //Value used when testing on Rev Client
  /** Creates a new Carousel. */
  public Carousel() {
    carousel = new SparkFlex(CANIDS.carousel, MotorType.kBrushless);
    carousel_Config = new SparkFlexConfig();

    carousel_Encoder = carousel.getEncoder();
    carousel_ClosedLoopController = carousel.getClosedLoopController();

    carousel_Config.closedLoop
    .pid(CarouselConstants.p, CarouselConstants.i, CarouselConstants.d);

    carousel.configure(carousel_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

    /**Sets the speed for the wheels using PID controller on velocity control type
   * @param Speed in RPM
   */
  public void setSpeed(double Speed){
    carousel_ClosedLoopController.setSetpoint(Speed, ControlType.kVelocity);
  }
  public void dashboardSetSpeed(){
    carousel_ClosedLoopController.setSetpoint(SmartDashboard.getNumber("Command Speed", speed), ControlType.kVelocity);
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Carousel Speed (RPM)", carousel_Encoder.getVelocity());
    SmartDashboard.putNumber("Carousel Command Speed", speed);
  }
}
