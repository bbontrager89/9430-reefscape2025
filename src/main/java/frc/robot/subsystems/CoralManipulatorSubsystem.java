// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;

public class CoralManipulatorSubsystem extends SubsystemBase {

  private SparkFlex pivotMotor = new SparkFlex(CoralManipulatorConstants.coralManipulatorPivotMotorCanid, MotorType.kBrushless);
  private SparkFlex intakeMotor = new SparkFlex(CoralManipulatorConstants.coralManipulatorIntakeMotorCanid, MotorType.kBrushless);
  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {}

  public void setPivotMotorSpeed(double speed){
    pivotMotor.set(speed);
  }

  public void setIntakeMotorSpeed(double speed){
    intakeMotor.set(speed);
  }

  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }

  public void stopIntakeMotor(){
    intakeMotor.stopMotor();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
