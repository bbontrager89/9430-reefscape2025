// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.utils.Elastic;

public class CoralManipulatorSubsystem extends SubsystemBase {

  // ---------- Pivot Motor Fields ----------
  private final SparkMax pivotMotor = new SparkMax(CoralManipulatorConstants.PivotMotorCanId, MotorType.kBrushless);
  private final SparkAbsoluteEncoder pivotEncoder;
  private final PIDController pivotPIDController;
  // A normalized value (e.g., 0.0 to 1.0) for the pivot’s target position.
  private double desiredPivotPosition = 0.25;

  private boolean autoMode = true;

  private static SendableChooser<Command> pivotCommands;

  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {
    // Pivot motor setup
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotController = new PIDController(CoralManipulatorConstants.pivotKp, CoralManipulatorConstants.pivotKi, CoralManipulatorConstants.pivotKd);

    pivotController.setTolerance(0);
    pivotController.setIntegratorRange(0.13, 0.47);
  }

  /**
   * Configures SmartDashboard controls for pivot commands.
   */
  public void configureDashboardControls() {
    pivotCommandChooser = new SendableChooser<>();

    pivotCommandChooser.setDefaultOption("P 0.2", new InstantCommand(() -> movePivotTo(0.2)));
    pivotCommandChooser.addOption("P 0.3", new InstantCommand(() -> movePivotTo(0.3)));
    pivotCommandChooser.addOption("P 0.4", new InstantCommand(() -> movePivotTo(0.4)));
    pivotCommandChooser.addOption("P min", new InstantCommand(() -> movePivotTo(0.11)));
    pivotCommandChooser.addOption("P max", new InstantCommand(() -> movePivotTo(0.49)));
    pivotCommandChooser.addOption("P Custom", new InstantCommand(() -> movePivotTo(
        SmartDashboard.getNumber("Custom Pivot Height", 0.2))));

    SmartDashboard.putData("Pivot Height Commands", pivotCommandChooser);
    SmartDashboard.putData("Run Pivot Command", new ScheduleCommand(
        new InstantCommand(() -> {
          Command cmd = pivotCommandChooser.getSelected();
          if (cmd != null) {
            cmd.schedule();
          }
        })
    ));

    SmartDashboard.putNumber("Custom Pivot Height", 0.2);
    SmartDashboard.putNumber("Desired Pivot Height", desiredPivotPosition);
  }

  // ================= Pivot Motor Control Methods =================

  /**
   * Sets the pivot motor speed and updates the active flag.
   *
   * @param speed The motor output.
   */
  private void setPivotMotorSpeed(double speed) {
    pivotMotor.set(speed);
    isPivotMotorActive = (speed != 0);
  }

  /**
   * Allows manual control of the pivot motor speed.
   *
   * @param speed Motor output speed.
   */
  public void startPivotMotor(double speed) {
    setPivotMotorSpeed(speed);
  }

  /**
   * Stops the pivot motor.
   */
  public void stopPivotMotor() {
    pivotMotor.stopMotor();
    isPivotMotorActive = false;
  }

  /**
   * Sets the desired pivot position (normalized) ensuring it stays within limits.
   *
   * @param position The target position.
   */
  public void movePivotTo(double position) {
    desiredPivotPosition = Math.min(Math.max(position,
        CoralManipulatorConstants.minimumPivotPosition),
        CoralManipulatorConstants.maximumPivotPosition);
    SmartDashboard.putNumber("Desired Pivot Height", desiredPivotPosition);
    // (Optionally) reset or reinitialize the PID controller here.
    pivotPIDController.reset();
  }

  /**
   * Updates the desired pivot position to the current encoder reading.
   * Useful when manually adjusting the pivot.
   */
  public void updateDesiredPivotPosition() {
    desiredPivotPosition = getPivotMotorPosition();
    pivotPIDController.reset();
  }

  /**
   * Returns the current pivot motor encoder position.
   *
   * @return The encoder reading.
   */
  public double getPivotMotorPosition() {
    return pivotEncoder.getPosition();
  }

  /**
   * Returns the current pivot motor velocity.
   *
   * @return The encoder-reported velocity.
   */
  public double getPivotVelocity() {
    return pivotEncoder.getVelocity();
  }

  /**
   * Checks if the pivot is at the desired position.
   *
   * @return true if the error is within tolerance.
   */
  public boolean atPivotPosition() {
    return pivotPIDController.atSetpoint();
  }

  // ================= Intake Motor Control Methods =================

  /**
   * Sets the intake motor speed and updates diagnostics.
   *
   * @param speed The desired speed.
   */
  private void setIntakeMotorSpeed(double speed) {
    intakeMotor.set(speed);
    currentIntakeSpeed = speed;
    // Record the start time if transitioning from a stopped state.
    if (intakeState == ManipulatorState.Stopped) {
      intakeStartTime = Timer.getFPGATimestamp();
      isIntakeMotorActive = true;
    }
  }

  /**
   * Starts the intake motor at a given speed continuously.
   *
   * @param speed The motor speed.
   */
  public void startIntakeMotor(double speed) {
    intakeState = ManipulatorState.Active;
    setIntakeMotorSpeed(speed);
  }

  /**
   * Starts the intake motor for a fixed duration at a given speed.
   *
   * @param speed    The motor speed.
   * @param duration Time (in seconds) to run at full speed.
   */
  public void startIntakeMotor(double speed, double duration) {
    intakeAutoStopTime = Timer.getFPGATimestamp() + duration;
    intakeState = ManipulatorState.Auto;
    setIntakeMotorSpeed(speed);
  }

  /**
   * Starts the intake motor for a fixed duration at full speed, then decelerates over a given period.
   *
   * @param speed             The motor speed.
   * @param fullSpeedDuration Duration (in seconds) at full speed.
   * @param slowDuration      Duration (in seconds) to decelerate.
   */
  public void startIntakeMotor(double speed, double fullSpeedDuration, double slowDuration) {
    intakeAutoStopTime = Timer.getFPGATimestamp() + fullSpeedDuration;
    intakeSlowDuration = slowDuration;
    intakeState = ManipulatorState.Auto;
    setIntakeMotorSpeed(speed);
  }

  /**
   * Initiates deceleration of the intake motor over the specified duration.
   *
   * @param slowDuration Duration (in seconds) for deceleration.
   */
  public void slowIntakeMotor(double slowDuration) {
    // Compute a fixed decrement per cycle.
    // Assumes periodic() is called approximately 50 times per second.
    intakeSlowingStep = currentIntakeSpeed / (50 * slowDuration);
    intakeState = ManipulatorState.Slowing;
  }

  /**
   * Slows the intake motor over a default 1-second period.
   */
  public void slowIntakeMotor() {
    slowIntakeMotor(1.0);
  }

  /**
   * Immediately stops the intake motor and resets its state.
   */
  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
    intakeState = ManipulatorState.Stopped;
    intakeStartTime = 0.0;
    currentIntakeSpeed = 0.0;
    isIntakeMotorActive = false;
  }

  // ================= Periodic Loop =================

  @Override
  public void periodic() {
    updatePivotMotorControl();
    updateIntakeMotorControl();
    updateDashboardDiagnostics();
  }

  /**
   * Updates pivot motor output by combining PID control with a gravity feedforward.
   * The PID controller’s setpoint is updated, then its output is combined with the feedforward.
   * If the system is within tolerance, we still drive the motor with the feedforward value to hold position.
   */
  private void updatePivotMotorControl() {
    double currentPosition = getPivotMotorPosition();
    // Update the PID controller’s setpoint.
    pivotPIDController.setSetpoint(desiredPivotPosition);
    // Compute the PID output based on the current position.
    double pidOutput = pivotPIDController.calculate(currentPosition);
    // Combine the PID and feedforward terms.
    double output = pidOutput;
    
    // Adjust the sign as necessary based on motor wiring.
    setPivotMotorSpeed(-output);
  }

  /**
   * Manages the intake motor state machine: auto-running, deceleration, and current limiting.
   */
  private void updateIntakeMotorControl() {
    double currentTime = Timer.getFPGATimestamp();
    
    // Auto current-limiting: if current draw is too high, stop the motor.
    if (enableAutoCurrentLimit &&
        intakeMotor.getOutputCurrent() > CoralManipulatorConstants.autoStopCurrent) {
      stopIntakeMotor();
      Elastic.sendWarning("Intake Motor Current Spike", "Likely Coral Fully Intaken");
      return;
   * Returns the position of the pivot motor
   * 
   * @return double representing the position
   */
  public double getPivotVelocity() {
    return pivotEncoder.getVelocity();
  }

  public boolean atPivotPosition() {
    return pivotController.atSetpoint();
  }

  @Override
  public void periodic() {

    double intakeMotorUptime = (isIntakeMotorOn) ? Timer.getFPGATimestamp() - intakeOnTimestamp : 0.0;

    // Log Data
    SmartDashboard.putBoolean("Intake Motor Active", isIntakeMotorOn);
    SmartDashboard.putBoolean("Pivot Motor Active", isPivotMotorOn);
    SmartDashboard.putNumber("Pivot Motor Position", getPivotMotorPosition());
    SmartDashboard.putNumber("Pivot Motor Speed", getPivotVelocity());
    SmartDashboard.putNumber("Intake Motor Uptime", intakeMotorUptime);
    SmartDashboard.putNumber("Intake Motor Stop Time", autoStopTime);
    SmartDashboard.putNumber("Intake Motor Timestamp", intakeOnTimestamp);
    SmartDashboard.putNumber("Intake Motor Speed", intakeSpeed);
    SmartDashboard.putNumber("Pivot Error", pivotController.getError());

    if (autoMode)
      pivotMotor.set(Math.min(Math.max(-pivotController.calculate(getPivotMotorPosition(), desiredPivotPosition), -CoralManipulatorConstants.maxPivotSpeed),CoralManipulatorConstants.maxPivotSpeed));
    

    // Check if motor is stuck to prevent over straining it
    if (doAutoCurrentLimit) {
      if (intakeMotor.getOutputCurrent() > CoralManipulatorConstants.autoStopCurrent) {
        stopIntakeMotor();
        Elastic.sendWarning("Pivot Motor Current Spike", "Likely Coral Fully Intaken");
      }
    }
    
    // Transition from Auto to Slowing based on time.
    if (intakeState == ManipulatorState.Auto && currentTime > intakeAutoStopTime) {
      slowIntakeMotor(intakeSlowDuration);
    }
    
    // Gradually reduce intake speed when in the Slowing state.
    if (intakeState == ManipulatorState.Slowing) {
      double newSpeed = currentIntakeSpeed - intakeSlowingStep;
      setIntakeMotorSpeed(newSpeed);
      // Stop when speed is near zero.
      if (Math.abs(newSpeed) < 0.05) {
        stopIntakeMotor();
      }
    }
  }

  /**
   * Publishes diagnostic data to the SmartDashboard.
   */
  private void updateDashboardDiagnostics() {
    double currentPivotPosition = getPivotMotorPosition();
    double pivotError = pivotPIDController.getError();
    double intakeRunTime = isIntakeMotorActive ? Timer.getFPGATimestamp() - intakeStartTime : 0.0;

    SmartDashboard.putBoolean("Intake Motor Active", isIntakeMotorActive);
    SmartDashboard.putBoolean("Pivot Motor Active", isPivotMotorActive);
    SmartDashboard.putNumber("Pivot Motor Position", currentPivotPosition);
    SmartDashboard.putNumber("Pivot Motor Velocity", getPivotVelocity());
    SmartDashboard.putNumber("Pivot Error", pivotError);
    SmartDashboard.putNumber("Intake Motor Uptime", intakeRunTime);
    SmartDashboard.putNumber("Intake Auto Stop Time", intakeAutoStopTime);
    SmartDashboard.putNumber("Intake Motor Start Timestamp", intakeStartTime);
    SmartDashboard.putNumber("Intake Motor Speed", currentIntakeSpeed);
  }

  // ================= Internal State Enum =================

  private enum ManipulatorState {
    Active,   // Running at a set speed manually.
    Auto,     // Running automatically until a timer expires.
    Slowing,  // In deceleration mode.
    Stopped   // Motor is stopped.
  }
}
