// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingArmConstants;

public class ClimbingArmSubsystem extends SubsystemBase {

  private SparkFlex motor1 = new SparkFlex(ClimbingArmConstants.motor1CanId, MotorType.kBrushless);
  private SparkFlex motor2 = new SparkFlex(ClimbingArmConstants.motor2CanId, MotorType.kBrushless);

  private SparkAbsoluteEncoder encoder;

  /** Creates a new ClimbingArmSubsystem. */
  public ClimbingArmSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
