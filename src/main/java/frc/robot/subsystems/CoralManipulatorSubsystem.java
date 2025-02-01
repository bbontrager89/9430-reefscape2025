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
  private boolean autoIntake = false;
  private Double autoStopTime = null;

  private boolean isIntakeMotorOn;
  private boolean isPivotMotorOn;

  private double intakeOnTimestamp = Double.NEGATIVE_INFINITY;



  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {
  }

  public void setPivotMotorSpeed(double speed) {
    pivotMotor.set(speed);
    isPivotMotorOn = true;
  }

  public void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
    isIntakeMotorOn = true;
    intakeOnTimestamp = Timer.getFPGATimestamp();
  }

  public void stopPivotMotor() {
    pivotMotor.stopMotor();
    isPivotMotorOn = false;
  }

  public void stopIntakeMotor() {
    intakeMotor.stopMotor();

    isIntakeMotorOn = false;
    autoIntake = false;
  }

  public double getIntakeMotorPosition() {
    return intakePosEncoder.getPosition();
  }

  public void runIntakeFor(double speed, double time) {
    autoStopTime = time + Timer.getFPGATimestamp();
    setIntakeMotorSpeed(speed);
    autoIntake = true;
  }

  @Override
  public void periodic() {
    double intakeMotorUptime = Timer.getFPGATimestamp() - intakeOnTimestamp; 

    // Log Data
    SmartDashboard.putBoolean("Intake Motor Active", isIntakeMotorOn);
    SmartDashboard.putBoolean("Pivot Motor Active", isPivotMotorOn);
    SmartDashboard.putNumber("Intake Motor Position", getIntakeMotorPosition());

    // Check if motor is stuck to prevent over straining it
    if (isIntakeMotorOn && doAutoCurrentLimit && 
        intakeMotorUptime > CoralManipulatorConstants.currentSpikeCheckDelay) {
      if (intakeMotor.getOutputCurrent() > CoralManipulatorConstants.autoStopCurrent) {
        stopIntakeMotor();
      }
    }

    // Turn off motor after time has passed
    if (autoIntake && autoStopTime != null) {
      if (Timer.getFPGATimestamp() > autoStopTime) {
        stopIntakeMotor();
      }
    }


  }
}
