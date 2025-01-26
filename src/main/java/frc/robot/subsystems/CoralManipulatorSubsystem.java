// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;

public class CoralManipulatorSubsystem extends SubsystemBase {

  // private SparkMax pivotMotor = new SparkMax(CoralManipulatorConstants.coralManipulatorPivotMotorCanid, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(CoralManipulatorConstants.coralManipulatorIntakeMotorCanid, MotorType.kBrushless);
  private RelativeEncoder intakePosEncoder = intakeMotor.getEncoder();
  private double lastKnownPosition;
  private boolean isIntakeMotorOn;

  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {}

  public void setPivotMotorSpeed(double speed){
    // pivotMotor.set(speed);
  }

  public void setIntakeMotorSpeed(double speed){
    intakeMotor.set(speed);
    isIntakeMotorOn = true;
  }

  public void stopPivotMotor(){
    // pivotMotor.stopMotor();
  }

  public void stopIntakeMotor(){
    intakeMotor.stopMotor();
    isIntakeMotorOn = false;
  }

  public double getIntakeMotorPosition(){
    return intakePosEncoder.getPosition();   
  }

  public void runIntakeFor(double speed, double time){
    setIntakeMotorSpeed(speed);
    isIntakeMotorOn = true;
    Timer.delay(time);
    stopIntakeMotor();
    isIntakeMotorOn = false;
  }
  @Override
  public void periodic() {
    if(isIntakeMotorOn){
    double dm = getIntakeMotorPosition() - lastKnownPosition;

    if(Math.abs(dm) > 0.005) {
      lastKnownPosition = getIntakeMotorPosition();
    }
    else{
      stopIntakeMotor();
    }
  }

  }
}
