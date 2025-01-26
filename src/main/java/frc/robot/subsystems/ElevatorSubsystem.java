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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex elevatorMotor = new SparkFlex(ElevatorConstants.elevatorMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder absoluteEncoder = elevatorMotor.getAbsoluteEncoder();

  private SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

  private double desiredHeight;
  private double autoSpeed = 0.0;
  private boolean autoMode = false;

  private boolean aboveMaxHeight = false;
  private boolean belowMinHeight = false;
  private double currentSpeed = 0.0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    // Calculates the soft limits in revolutions based on
    // the initial reading of the absolute encoder
    double lowerSoftLimit = (absoluteEncoder.getPosition() - ElevatorConstants.minimumElevatorHeight)
        * ElevatorConstants.encoderToRevolutionRatio;

    // Upper limit by the range of revolutions from the lower limit
    double upperSoftLimit = (lowerSoftLimit - ElevatorConstants.rangeInRevolutions);

    SoftLimitConfig softLimitConfig = new SoftLimitConfig()
        .forwardSoftLimit(lowerSoftLimit)
        .reverseSoftLimit(upperSoftLimit)
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);

    elevatorMotorConfig.inverted(ElevatorConstants.elevatorMotorInverted);
    elevatorMotorConfig.apply(softLimitConfig);

    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setMotorSpeed(double speed) {
    currentSpeed = speed;
    if (!(aboveMaxHeight && (speed < 0)) && !(belowMinHeight && (speed > 0))) {
      elevatorMotor.set(speed);
    }
  }

  public void stopMotor() {
    currentSpeed = 0.0;
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
      case 5:
        desiredHeight = ElevatorConstants.level5ScoringPosition;
        break;
      default:
        System.out.println("Invalid Scoring Position requested in ElevatorSubsystem");
        break;
    }

    // ;;;;;;;;desiredHeight=(scoringPosition==1)?ElevatorConstants.level1ScoringPosition:(scoringPosition==2)?ElevatorConstants.level3ScoringPosition:(scoringPosition==3)?ElevatorConstants.level3ScoringPosition/*IWouldNotRecommend-Titus(ButItIsFunny)*/:(scoringPosition==4)?ElevatorConstants.level4ScoringPosition:-1.0;;;;;;;;;;;;;;;;;;;;;;;;;;;

    autoMode = true;

  }

  public void turnOffAutoMode() {
    autoMode = false;
    stopMotor();
  }

  @Override
  public void periodic() {

    // Log data
    SmartDashboard.putNumber("Elevator Position", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Velocity", absoluteEncoder.getVelocity());

    // Lower soft limit check
    if (absoluteEncoder.getPosition() > ElevatorConstants.maximumElevatorHeight) {
      if (currentSpeed < 0) {
        stopMotor();
      }
      if (autoSpeed < 0 && autoMode) {
        turnOffAutoMode();
      }
      aboveMaxHeight = true;
    } else {
      aboveMaxHeight = false;
    }

    // Upper soft limit check
    if (absoluteEncoder.getPosition() < ElevatorConstants.minimumElevatorHeight) {
      if (currentSpeed > 0) {
        stopMotor();
      }

      if (autoSpeed > 0 && autoMode) {
        turnOffAutoMode();
      }

      belowMinHeight = true;

    } else {
      belowMinHeight = false;
    }

    // Run auto movement
    if (autoMode) {

      // Calculate error
      double elevatorError = (desiredHeight - absoluteEncoder.getPosition());

      // Adjust speed to error
      double autoSpeed = ElevatorConstants.kP * elevatorError;
      autoSpeed = (autoSpeed > 1) ? 1 : (autoSpeed < -1) ? -1 : autoSpeed;

      // Log data
      SmartDashboard.putNumber("AutoSpeed", autoSpeed);

      // Run or don't run
      if (Math.abs(elevatorError) < ElevatorConstants.positionTolerence) {
        turnOffAutoMode();
      } else {
        setMotorSpeed(autoSpeed);
      }

    }

  }
}
