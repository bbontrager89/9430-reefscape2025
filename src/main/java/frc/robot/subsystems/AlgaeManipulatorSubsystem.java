// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeManipulatorSubsystem extends SubsystemBase {

  private SparkFlex pivotMotor = new SparkFlex(AlgaeConstants.pivotMotorCANid, MotorType.kBrushless);
  private SparkFlex intakeMotor = new SparkFlex(AlgaeConstants.intakeMotorCANid, MotorType.kBrushless);

  private SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private Double desiredPivotHeight;

  /** Creates a new AlgaeManipulatorSubSystem. */
  public AlgaeManipulatorSubsystem() {
    
  }

  public void setPivotSpeed(double speed) {
    pivotMotor.set(speed);
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void setDesiredPivotHeight(double height) {
    desiredPivotHeight = height;
  }

  public void disableAutoPivot() {
    desiredPivotHeight = null;
  }

  public double getPivotEncoderReading() {
    return pivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (desiredPivotHeight != null) {
      // PID controller to position
    }

  }

  }
}
