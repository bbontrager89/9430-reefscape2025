// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex elevatorMotor = new SparkFlex(ElevatorConstants.elevatorMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder absoluteEncoder;
  private PIDController elevatorPIDController;

  private double desiredHeight;
  private double initialSpeed = 0.5;
  private double currentSpeed = 0.0;
  private boolean movingToScoringPosition = false;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    absoluteEncoder = elevatorMotor.getAbsoluteEncoder();

    SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

    SoftLimitConfig softLimitConfig = new SoftLimitConfig()
        .forwardSoftLimit(50)
        .reverseSoftLimit(50)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    elevatorMotorConfig.apply(softLimitConfig);

    // elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorPIDController = new PIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD);
    
    elevatorPIDController.setIntegratorRange(0.01, 4.7);
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
        desiredHeight = ElevatorConstants.level1ScoringPosition;
        break;
      case 2:
        desiredHeight = ElevatorConstants.level2ScoringPosition;
        break;
      case 3:
        desiredHeight = ElevatorConstants.level3ScoringPosition;
        break;
      case 4:
        desiredHeight = ElevatorConstants.level4ScoringPosition;
        break;
      default:
        System.out.println("Invalid Scoring Position requested in ElevatorSubsystem");
        break;
    }

    currentSpeed = initialSpeed;
    movingToScoringPosition = true;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    System.out.println(elevatorMotor.getAppliedOutput());

    /* 

    SmartDashboard.putBoolean("mtsp", movingToScoringPosition);

    if (absoluteEncoder.getPosition() < 0.1 || absoluteEncoder.getPosition() > 0.4) {
      currentSpeed = initialSpeed / 2.0;
      if (absoluteEncoder.getPosition() < 0.5 || absoluteEncoder.getPosition() > 0.45) {
        currentSpeed = initialSpeed / 4.0;
        if (absoluteEncoder.getPosition() < 0.25 || absoluteEncoder.getPosition() > 0.475) {
          currentSpeed = 0.0;
        }
      }
    } else {
      currentSpeed = initialSpeed;
    }

    if (movingToScoringPosition) {

      if (absoluteEncoder.getPosition() > desiredHeight - 0.5 && absoluteEncoder.getPosition() < desiredHeight + 0.05) {

        stopMotor();
        movingToScoringPosition = false;

      } else if (absoluteEncoder.getPosition() > desiredHeight) {

        if (currentSpeed > 0) {
          currentSpeed = -Math.abs(currentSpeed) / 2.0;
        }

        setMotorSpeed(currentSpeed);

      } else if (absoluteEncoder.getPosition() < desiredHeight) {

        if (currentSpeed < 0) {
          currentSpeed = Math.abs(currentSpeed) / 2.0;
        }

        setMotorSpeed(currentSpeed);

      }

    }
      */

  }
}
