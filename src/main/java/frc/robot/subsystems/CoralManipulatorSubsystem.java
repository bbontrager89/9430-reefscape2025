// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.utils.Elastic;

public class CoralManipulatorSubsystem extends SubsystemBase {

  // Pivot and intake motors
  private SparkMax pivotMotor = new SparkMax(CoralManipulatorConstants.PivotMotorCanId, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(CoralManipulatorConstants.IntakeMotorCanId, MotorType.kBrushless);

  // Encoder and PID for pivot control
  private SparkAbsoluteEncoder pivotEncoder;
  private PIDController pivotController;

  // Intake motor timing and state variables
  private boolean doAutoCurrentLimit = true;
  private double autoStopTime = Double.POSITIVE_INFINITY;
  private double slowingFactor = 0.1;
  private double slowTime = 0.0;

  private double intakeSpeed = 0.0;
  private boolean isIntakeMotorOn = false;
  private boolean isPivotMotorOn = false;

  private CoralManipulatorState intakeState = CoralManipulatorState.Stopped;
  private double intakeOnTimestamp = Double.NEGATIVE_INFINITY;

  // Desired pivot position (normalized value, e.g., 0.0 to 1.0)
  private double desiredPivotPosition = 0.25;

  private static SendableChooser<Command> pivotCommands;

  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {
    pivotEncoder = pivotMotor.getAbsoluteEncoder();

    // Adjusted PID gains for smoother, less oscillatory motion:
    // - Lower proportional gain to reduce overreaction to manual disturbances.
    // - Increased derivative gain to dampen oscillations.
    pivotController = new PIDController(1.0, 0.0, 0.1);
    pivotController.setTolerance(0.005);
    pivotController.setIntegratorRange(-0.2, 0.2);
  }

  /**
   * Configures dashboard controls for pivot commands.
   */
  public void configureDashboardControls() {

    pivotCommands = new SendableChooser<Command>();

    pivotCommands.setDefaultOption("P 0.2", new InstantCommand(() -> {
      movePivotTo(0.2);
    }));
    pivotCommands.addOption("P 0.3", new InstantCommand(() -> {
      movePivotTo(0.3);
    }));
    pivotCommands.addOption("P 0.4", new InstantCommand(() -> {
      movePivotTo(0.4);
    }));
    pivotCommands.addOption("P min", new InstantCommand(() -> {
      movePivotTo(0.11);
    }));
    pivotCommands.addOption("P max", new InstantCommand(() -> {
      movePivotTo(0.49);
    }));
    pivotCommands.addOption("P Custom", new InstantCommand(() -> {
      movePivotTo(SmartDashboard.getNumber("Custom Pivot Height", 0.2));
    }));

    SmartDashboard.putData("Pivot Height Commands", pivotCommands);

    SmartDashboard.putData("Run Pivot Command", new ScheduleCommand(new InstantCommand(() -> {
      if (pivotCommands.getSelected() != null)
        pivotCommands.getSelected().schedule();
    })));

    SmartDashboard.putNumber("Custom Pivot Height", 0.2);
    SmartDashboard.putNumber("Desired Pivot Height", 0.0);
  }

  // ================= Pivot Motor Control =================

  /**
   * Sets the pivot motor speed and updates the status flag.
   * 
   * @param speed the speed to set the motor
   */
  private void setPivotMotorSpeed(double speed) {
    pivotMotor.set(speed);
    isPivotMotorOn = true;
  }

  /**
   * Public method to start the pivot motor at a given speed.
   * 
   * @param speed the speed to set the motor
   */
  public void startPivotMotor(double speed) {
    setPivotMotorSpeed(speed);
  }

  /**
   * Stops the pivot motor immediately.
   */
  public void stopPivotMotor() {
    pivotMotor.stopMotor();
    isPivotMotorOn = false;
  }

  /**
   * Sets the desired pivot position. Ensures the position is within allowed limits.
   * 
   * @param pos the target position
   */
  public void movePivotTo(double pos) {
    desiredPivotPosition = pos;
    if (desiredPivotPosition > CoralManipulatorConstants.maximumPivotPosition) {
      desiredPivotPosition = CoralManipulatorConstants.maximumPivotPosition;
    }
    if (desiredPivotPosition < CoralManipulatorConstants.minimumPivotPosition) {
      desiredPivotPosition = CoralManipulatorConstants.minimumPivotPosition;
    }
    SmartDashboard.putNumber("Desired Pivot Height", desiredPivotPosition);
  }

  /**
   * If the pivot is manually moved, call this method to update the desired
   * position so the PID doesn't fight the manual adjustment.
   */
  public void updateDesiredPivotPosition() {
    desiredPivotPosition = getPivotMotorPosition();
    pivotController.reset();
  }

  /**
   * Calculates a simple gravity feedforward term based on the current pivot position.
   * 
   * @param position the current pivot position
   * @return the feedforward output to help counteract gravity
   */
  private double calculateGravityFeedForward(double position) {
    double kGravity = 0.1;  // Tune this constant as needed.
    // Assume position is normalized [0, 1] and mapped to an angle in radians.
    double angleRadians = position * Math.PI;
    return kGravity * Math.cos(angleRadians);
  }

  // ================= Intake Motor Control =================

  /**
   * Sets the intake motor speed and updates the state.
   * 
   * @param speed the speed to set the motor
   */
  private void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
    intakeSpeed = speed;

    if (intakeState == CoralManipulatorState.Stopped) {
      intakeOnTimestamp = Timer.getFPGATimestamp();
      isIntakeMotorOn = true;
    }
  }

  /**
   * Starts the intake motor at the given speed.
   * 
   * @param speed the speed to set the motor
   */
  public void startIntakeMotor(double speed) {
    intakeState = CoralManipulatorState.Active;
    setIntakeMotorSpeed(speed);
  }

  /**
   * Starts the intake motor for a specified time.
   * 
   * @param speed the speed to set the motor
   * @param time  the duration to run the motor
   */
  public void startIntakeMotor(double speed, double time) {
    autoStopTime = time + Timer.getFPGATimestamp();
    setIntakeMotorSpeed(speed);
    intakeState = CoralManipulatorState.Auto;
  }

  /**
   * Starts the intake motor for a specified time with a slowing period afterward.
   * 
   * @param speed       the speed to set the motor
   * @param time        the duration to run the motor at full speed
   * @param slowingTime the duration over which the motor slows down to a stop
   */
  public void startIntakeMotor(double speed, double time, double slowingTime) {
    autoStopTime = time + Timer.getFPGATimestamp();
    slowTime = slowingTime;
    setIntakeMotorSpeed(speed);
    intakeState = CoralManipulatorState.Auto;
  }

  /**
   * Slows the intake motor to zero over a default 1-second interval.
   */
  public void slowIntakeMotor() {
    slowIntakeMotor(1);
  }

  /**
   * Slows the intake motor to zero over the specified time interval.
   * 
   * @param time time in seconds before the motor stops
   */
  public void slowIntakeMotor(double time) {
    slowingFactor = (intakeSpeed / (50 * time));
    intakeState = CoralManipulatorState.Slowing;
  }

  /**
   * Stops the intake motor immediately.
   */
  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
    intakeState = CoralManipulatorState.Stopped;
    intakeOnTimestamp = 0.0;
    intakeSpeed = 0.0;
    isIntakeMotorOn = false;
  }

  /**
   * Returns the current pivot motor position.
   * 
   * @return the pivot position
   */
  public double getPivotMotorPosition() {
    return pivotEncoder.getPosition();
  }

  /**
   * Returns the current pivot motor velocity.
   * 
   * @return the pivot velocity
   */
  public double getPivotVelocity() {
    return pivotEncoder.getVelocity();
  }

  /**
   * Checks whether the pivot has reached its setpoint.
   * 
   * @return true if the pivot is at the desired position
   */
  public boolean atPivotPosition() {
    return pivotController.atSetpoint();
  }

  // ================= Periodic Loop =================

  @Override
  public void periodic() {
    // ----- Pivot Motor Control -----
    double currentPosition = getPivotMotorPosition();
    double error = desiredPivotPosition - currentPosition;

    // Deadband: if the error is very small, do not command the motor.
    if (Math.abs(error) < pivotController.getErrorTolerance()) {
      pivotMotor.set(0);
    } else {
      double pidOutput = pivotController.calculate(currentPosition, desiredPivotPosition);
      double feedForward = calculateGravityFeedForward(currentPosition);
      // The negative sign may be necessary based on your motor wiring/direction.
      pivotMotor.set(-(pidOutput + feedForward));
    }

    // ----- Intake Motor Control & Diagnostics -----
    double intakeMotorUptime = (isIntakeMotorOn) ? Timer.getFPGATimestamp() - intakeOnTimestamp : 0.0;

    SmartDashboard.putBoolean("Intake Motor Active", isIntakeMotorOn);
    SmartDashboard.putBoolean("Pivot Motor Active", isPivotMotorOn);
    SmartDashboard.putNumber("Pivot Motor Position", currentPosition);
    SmartDashboard.putNumber("Pivot Motor Speed", getPivotVelocity());
    SmartDashboard.putNumber("Intake Motor Uptime", intakeMotorUptime);
    SmartDashboard.putNumber("Intake Motor Stop Time", autoStopTime);
    SmartDashboard.putNumber("Intake Motor Timestamp", intakeOnTimestamp);
    SmartDashboard.putNumber("Intake Motor Speed", intakeSpeed);
    SmartDashboard.putNumber("Pivot Error", pivotController.getError());

    // Check if the intake motor current is too high (to prevent stalling)
    if (doAutoCurrentLimit) {
      if (intakeMotor.getOutputCurrent() > CoralManipulatorConstants.autoStopCurrent) {
        stopIntakeMotor();
        Elastic.sendWarning("Pivot Motor Current Spike", "Likely Coral Fully Intaken");
      }
    }

    // If running in Auto mode, check the timer to begin slowing down the intake.
    if (intakeState == CoralManipulatorState.Auto) {
      if (Timer.getFPGATimestamp() > autoStopTime) {
        slowIntakeMotor(slowTime);
      }
    }

    // Gradually reduce the intake speed when in Slowing mode.
    if (intakeState == CoralManipulatorState.Slowing) {
      double preSpeed = intakeSpeed;
      setIntakeMotorSpeed(intakeSpeed - slowingFactor); // reduce speed gradually
      if (Math.abs(intakeSpeed) < 0.05 || (preSpeed > 0 ? intakeSpeed < 0 : intakeSpeed > 0)) {
        stopIntakeMotor();
      }
    }
  }

  // ================= Internal State =================

  private enum CoralManipulatorState {
    Active,
    Auto,
    Slowing,
    Stopped
  }
}
