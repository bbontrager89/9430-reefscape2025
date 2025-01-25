// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;

public class CoralManipulatorSubsystem extends SubsystemBase {

  private SparkMax pivotMotor = new SparkMax(CoralManipulatorConstants.coralManipulatorPivotMotorCanid, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(CoralManipulatorConstants.coralManipulatorIntakeMotorCanid, MotorType.kBrushless);

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

  public void intake(){
    setIntakeMotorSpeed(-.1);
  }
  public void outtake(){
    setIntakeMotorSpeed(.1);
  }
  @Override
  public void periodic() {
    
  }
}
