// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex elevatorMotor = new SparkFlex(ElevatorConstants.elevatorMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder absoluteEncoder = elevatorMotor.getAbsoluteEncoder();
  private SparkClosedLoopController closedLoopController;
  private SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

  private double desiredHeight;
  private double initialSpeed = 0.1;
  private double currentSpeed = 0.0;
  private boolean movingToScoringPosition = false;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    // Calculates the soft limits in revolutions based on
    // the initial reading of the absolute encoder
    double lowerSoftLimit = (absoluteEncoder.getPosition() - ElevatorConstants.minimumElevatorHeight)
        * ElevatorConstants.encoderToRevolutionRatio;

    // Upper limit by the range of revolutions from the lower limit
    double upperSoftLimit = (lowerSoftLimit + ElevatorConstants.rangeInRevolutions);

    SoftLimitConfig softLimitConfig = new SoftLimitConfig()
        .forwardSoftLimit(lowerSoftLimit)
        .reverseSoftLimit(upperSoftLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true);

    elevatorMotorConfig.apply(ElevatorConstants.closedLoopConfig);
    elevatorMotorConfig.inverted(ElevatorConstants.elevatorMotorInverted);
    elevatorMotorConfig.apply(softLimitConfig);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = elevatorMotor.getClosedLoopController();

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

    // ;;;;;;;;desiredHeight=(scoringPosition==1)?ElevatorConstants.level1ScoringPosition:(scoringPosition==2)?ElevatorConstants.level3ScoringPosition:(scoringPosition==3)?ElevatorConstants.level3ScoringPosition/*IWouldNotRecommend-Titus(ButItIsFunny)*/:(scoringPosition==4)?ElevatorConstants.level4ScoringPosition:-1.0;;;;;;;;;;;;;;;;;;;;;;;;;;;

    closedLoopController.setReference(desiredHeight, ControlType.kPosition);

    currentSpeed = initialSpeed;
    movingToScoringPosition = true;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Elevator Position", absoluteEncoder.getPosition());

  }
}
