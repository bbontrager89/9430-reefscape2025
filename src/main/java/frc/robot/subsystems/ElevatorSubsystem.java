// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex elevatorMotor = new SparkFlex(Constants.elevatorMotorCANid, MotorType.kBrushless);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
  }

  public void setMotorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void stopMotor() {
    elevatorMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
