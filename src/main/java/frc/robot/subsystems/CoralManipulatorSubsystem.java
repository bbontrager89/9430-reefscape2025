// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;

public class CoralManipulatorSubsystem extends SubsystemBase {

  private SparkMax pivotMotor = new SparkMax(CoralManipulatorConstants.PivotMotorCanId, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(CoralManipulatorConstants.IntakeMotorCanId, MotorType.kBrushless);

  private RelativeEncoder intakePosEncoder = intakeMotor.getEncoder();

  private boolean doAutoCurrentLimit = false;
  private double autoStopTime = Double.POSITIVE_INFINITY;
  private double slowingFactor = 0.02;

  private double intakeSpeed = 0.0;

  private boolean isIntakeMotorOn;
  private boolean isPivotMotorOn;

  private CoralManipulatorState intakeState = CoralManipulatorState.Stopped;

  private double intakeOnTimestamp = Double.NEGATIVE_INFINITY;

  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {
  }

  @SuppressWarnings("unused")
  private void setPivotMotorSpeed(double speed) {
    pivotMotor.set(speed);
    isPivotMotorOn = true;
  }

  public void startPivotMotor(double speed) {
    
  }

  public void stopPivotMotor() {
    pivotMotor.stopMotor();
    isPivotMotorOn = false;
  }

  

  private void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
    isIntakeMotorOn = true;
    intakeSpeed = speed;
  }

  public void startIntakeMotor(double speed) {
    intakeState = CoralManipulatorState.Active;
    setIntakeMotorSpeed(speed);
  }

  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
    intakeState = CoralManipulatorState.Stopped;

    intakeOnTimestamp = 0.0;
    intakeSpeed = 0.0;
    isIntakeMotorOn = false;
  }

  public void slowIntakeMotor() {
    intakeState = CoralManipulatorState.Slowing;
    slowingFactor = 0.1;
  }

  public void slowIntakeMotor(double time) {
    intakeState = CoralManipulatorState.Slowing;
    slowingFactor = (50.0 / time) / 500.0;
  }

  public double getIntakeMotorPosition() {
    return intakePosEncoder.getPosition();
  }

  public void startIntakeMotor(double speed, double time) {
    autoStopTime = time + Timer.getFPGATimestamp();
    setIntakeMotorSpeed(speed);
    intakeState = CoralManipulatorState.Auto;
    intakeOnTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {

    double intakeMotorUptime = (isIntakeMotorOn) ? Timer.getFPGATimestamp() - intakeOnTimestamp : 0.0;

    // Log Data
    SmartDashboard.putBoolean("Intake Motor Active", isIntakeMotorOn);
    SmartDashboard.putBoolean("Pivot Motor Active", isPivotMotorOn);
    SmartDashboard.putNumber("Intake Motor Position", getIntakeMotorPosition());
    SmartDashboard.putNumber("Intake Motor Uptime", intakeMotorUptime);
    SmartDashboard.putNumber("Intake Motor Stop Time", autoStopTime);
    SmartDashboard.putNumber("Intake Motor Timestamp", intakeOnTimestamp);
    SmartDashboard.putNumber("Intake Motor Speed", intakeSpeed);

    // Check if motor is stuck to prevent over straining it
    if (isIntakeMotorOn && doAutoCurrentLimit &&
        intakeMotorUptime > CoralManipulatorConstants.currentSpikeCheckDelay) {
      if (intakeMotor.getOutputCurrent() > CoralManipulatorConstants.autoStopCurrent) {
        stopIntakeMotor();
      }
    }

    // Turn off motor after time has passed
    if (intakeState == CoralManipulatorState.Auto) {
      if (Timer.getFPGATimestamp() > autoStopTime) {
        slowIntakeMotor(1);
      }
    }

    if (intakeState == CoralManipulatorState.Slowing) {
      double preSpeed = intakeSpeed;
      setIntakeMotorSpeed(intakeSpeed - ((intakeSpeed > 0)? 1 : -1) * slowingFactor); // Slow motor as it approches stopping
      if (Math.abs(intakeSpeed) < 0.05 || (preSpeed > 0)? intakeSpeed < 0 : intakeSpeed > 0) {
        stopIntakeMotor();
      }
    }
  }
  
  public enum CoralManipulatorState {
      Active,
      Auto,
      Slowing,
      Stopped
  } 
}

