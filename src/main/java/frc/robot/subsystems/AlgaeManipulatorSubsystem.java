// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorConstants;

public class AlgaeManipulatorSubsystem extends SubsystemBase {

  private SparkFlex intakeMotor = new SparkFlex(AlgaeConstants.intakeMotorCANid, MotorType.kBrushless);
  private SparkFlex pivotMotor = new SparkFlex(AlgaeConstants.pivotMotorCANid, MotorType.kBrushless);

  private SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  private Double desiredPivotHeight = null;

  private boolean aboveMaxHeight = false;
  private boolean belowMinHeight = false;

  private double pivotSpeed = 0.0;

  /** Creates a new AlgaeManipulatorSubSystem. */
  public AlgaeManipulatorSubsystem() {
    /*
    // Configure pivot motor
    pivotMotor.configure(
      new SparkFlexConfig()
      .apply(
        new SoftLimitConfig()
          .forwardSoftLimit(AlgaeConstants.maximumPivotPosition)
          .forwardSoftLimitEnabled(true)
          .reverseSoftLimit(AlgaeConstants.minimumPivotPosition)
          .reverseSoftLimitEnabled(true))
      .apply(
        new ClosedLoopConfig()
          .pid(AlgaeConstants.kP,AlgaeConstants.kI,AlgaeConstants.kD))
      .idleMode(IdleMode.kBrake), 
    ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); */
  }

  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void setPivotSpeed(double speed) {
    if (!(aboveMaxHeight && speed < 0) && !(belowMinHeight && speed > 0)) {
      pivotMotor.set(speed);
      pivotSpeed = speed;
    }
  }

  public void stopPivot() {
    pivotMotor.stopMotor();
    pivotSpeed = 0.0;
  }

  public void setDesiredPivotHeight(double height) {
    desiredPivotHeight = height;
  }

  public void disableAutoPivot() {
    desiredPivotHeight = null;
  }

  public double getPivotHeight() {
    return pivotEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Lower soft limit check
    if (getPivotHeight() > AlgaeConstants.maximumPivotPosition) {
      if (pivotSpeed < 0) {
        stopPivot();
      }

      aboveMaxHeight = true;
      
    } else {
      aboveMaxHeight = false;
    }

    // Upper soft limit check
    if (getPivotHeight() < AlgaeConstants.minimumPivotPosition) {
      if (pivotSpeed > 0) {
        stopPivot();
      }

      belowMinHeight = true;

    } else {
      belowMinHeight = false;
    }

    if (desiredPivotHeight != null) {
      // PID controller to position

      // Calculate error
      double pivotError = (desiredPivotHeight - getPivotHeight());

      // Adjust speed to error
      double autoSpeed = AlgaeConstants.kP * pivotError;
      autoSpeed = (autoSpeed > AlgaeConstants.maximumAutoSpeed) ? AlgaeConstants.maximumAutoSpeed
          : (autoSpeed < -AlgaeConstants.maximumAutoSpeed) ? -AlgaeConstants.maximumAutoSpeed : autoSpeed;

      // Log data
      SmartDashboard.putNumber("Algae Pivot Error", pivotError);

      // Run or don't run
      if (Math.abs(pivotError) < AlgaeConstants.pivotTolerence) {
        disableAutoPivot();
      } else {
        setPivotSpeed(autoSpeed);
      }
    }

  }

  /** <B>Algae Position</B> enum for readability */
  public static enum AP {
    maximum(0),
    minimum(1),
    transit(2),
    intaking(3);

    public final int index;

    AP(int index) {
      this.index = index;
    }
  }
}
