// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex elevatorMotor = new SparkFlex(ElevatorConstants.elevatorMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder absoluteEncoder;

  private double desiredHeight; 
  final private double initialSpeed = 1.0;
  private double currentSpeed;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    absoluteEncoder = elevatorMotor.getAbsoluteEncoder();
  }

  public void setMotorSpeed(double speed) {
    elevatorMotor.set(speed);
  }

  public void stopMotor() {
    elevatorMotor.stopMotor();
  }

  public void moveToScoringPosition(int scoringPosition) {
    switch (scoringPosition) {
      case 1:
        desiredHeight = ElevatorConstants.levelOneScoringPosition;
        currentSpeed = initialSpeed;
        break;
      case 2:
        desiredHeight = ElevatorConstants.levelOneScoringPosition;
        currentSpeed = initialSpeed;
        break;
      case 3:
        desiredHeight = ElevatorConstants.levelOneScoringPosition;
        currentSpeed = initialSpeed;
        break;
      case 4:
        desiredHeight = ElevatorConstants.levelOneScoringPosition;
        currentSpeed = initialSpeed;
        break;
      default:
        System.out.println("Invalid Scoring Position requested in ElevatorSubsystem");
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (absoluteEncoder.getPosition() > desiredHeight && absoluteEncoder.getPosition() < desiredHeight + 0.01) {
      stopMotor();
    } else if (absoluteEncoder.getPosition() > desiredHeight) {
      currentSpeed = -Math.abs(currentSpeed/2);
      setMotorSpeed(currentSpeed);
    } else if (absoluteEncoder.getPosition() < desiredHeight) {
      currentSpeed = Math.abs(currentSpeed/2);
      setMotorSpeed(currentSpeed);
    }

  }
}
