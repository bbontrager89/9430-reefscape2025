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
import frc.robot.Constants.ElevatorConstants;
import frc.utils.Elastic;

public class CoralManipulatorSubsystem extends SubsystemBase {

  private SparkMax pivotMotor = new SparkMax(CoralManipulatorConstants.PivotMotorCanId, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(CoralManipulatorConstants.IntakeMotorCanId, MotorType.kBrushless);

  private SparkAbsoluteEncoder pivotEncoder;

  private PIDController pivotController;

  private boolean doAutoCurrentLimit = true;
  private double autoStopTime = Double.POSITIVE_INFINITY;
  private double slowingFactor = 0.1;
  private double slowTime = 0.0;

  private double intakeSpeed = 0.0;
  private boolean isIntakeMotorOn = false;
  private boolean isPivotMotorOn = false;

  private CoralManipulatorState intakeState = CoralManipulatorState.Stopped;

  private double intakeOnTimestamp = Double.NEGATIVE_INFINITY;

  private double desiredPivotPosition = 0.25;

  private static SendableChooser<Command> pivotCommands;

  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {

    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotController = new PIDController(CoralManipulatorConstants.pivotKp, CoralManipulatorConstants.pivotKi, CoralManipulatorConstants.pivotKd);

    pivotController.setTolerance(0.001);
    pivotController.setIntegratorRange(0.13, 0.47);
  }

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

  /**
   * Private method to set the pivot motor speed.
   * Accessed by methods inside the subsystem.
   * 
   * @param speed the speed to set the motor
   */
  private void setPivotMotorSpeed(double speed) {
    pivotMotor.set(speed);
    isPivotMotorOn = true;
  }

  /**
   * Public method to set the pivot motor speed.
   * 
   * @param speed the speed to set the motor
   */
  public void startPivotMotor(double speed) {
    setPivotMotorSpeed(speed);
  }

  /**
   * Stops the pivot motor immediatly.
   */
  public void stopPivotMotor() {
    pivotMotor.stopMotor();
    isPivotMotorOn = false;
  }

  public void movePivotTo(double pos) {
    desiredPivotPosition = pos;
    if (desiredPivotPosition > CoralManipulatorConstants.maximumPivotPosition) {
      desiredPivotPosition = CoralManipulatorConstants.maximumPivotPosition;
    }
    if (desiredPivotPosition < CoralManipulatorConstants.minimumPivotPosition) {
      desiredPivotPosition = CoralManipulatorConstants.minimumPivotPosition;
    }
    SmartDashboard.putNumber("Desired Pivot Height", desiredPivotPosition);
    pivotController.reset();
  }

  /**
   * Private method to set the intake motor speed.
   * Accessed by methods inside the subsystem.
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
   * Public method to set the intake motor speed.
   * Also sets the state of the motor to Active
   * 
   * @param speed the speed to set the motor
   */
  public void startIntakeMotor(double speed) {
    intakeState = CoralManipulatorState.Active;
    setIntakeMotorSpeed(speed);
  }

  /**
   * Public method to set the motor speed for a given time.
   * Accessed by methods inside the subsystem.
   * Slows down for 1 second after given time
   * 
   * @param speed the initial speed to set the motor
   * @param time  time to stay on for
   */
  public void startIntakeMotor(double speed, double time) {
    autoStopTime = time + Timer.getFPGATimestamp();
    setIntakeMotorSpeed(speed);
    intakeState = CoralManipulatorState.Auto;
  }

  /**
   * Public method to set the motor speed for a given time.
   * Accessed by methods inside the subsystem.
   * Slows down for 1 second after given time
   * 
   * @param speed       the initial speed to set the motor
   * @param time        time to stay on for
   * @param slowingTime time to slow down for after the run time
   */
  public void startIntakeMotor(double speed, double time, double slowingTime) {
    autoStopTime = time + Timer.getFPGATimestamp();
    slowTime = slowingTime;
    setIntakeMotorSpeed(speed);
    intakeState = CoralManipulatorState.Auto;
  }

  /**
   * Slows the motor to zero over a 1 second interval
   */
  public void slowIntakeMotor() {
    slowIntakeMotor(1);
  }

  /**
   * Slows the motor to zero over a time interval
   * 
   * @param time time in seconds before motor stops
   */
  public void slowIntakeMotor(double time) {
    slowingFactor = (intakeSpeed / (50 * time));
    intakeState = CoralManipulatorState.Slowing;
  }

  /**
   * Stops the intake motor immediatly.
   * Sets the intake state to stopped
   */
  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
    intakeState = CoralManipulatorState.Stopped;

    intakeOnTimestamp = 0.0;
    intakeSpeed = 0.0;
    isIntakeMotorOn = false;
  }

  /**
   * Returns the position of the pivot motor
   * 
   * @return double representing the position
   */
  public double getPivotMotorPosition() {
    return pivotEncoder.getPosition();
  }

  /**
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

    pivotMotor.set(Math.min(Math.max(-pivotController.calculate(getPivotMotorPosition(), desiredPivotPosition), -CoralManipulatorConstants.maxPivotSpeed),CoralManipulatorConstants.maxPivotSpeed));
    

    // Check if motor is stuck to prevent over straining it
    if (doAutoCurrentLimit) {
      if (intakeMotor.getOutputCurrent() > CoralManipulatorConstants.autoStopCurrent) {
        stopIntakeMotor();
        Elastic.sendWarning("Pivot Motor Current Spike", "Likely Coral Fully Intaken");
      }
    }

    // Turn off motor after time has passed
    if (intakeState == CoralManipulatorState.Auto) {
      if (Timer.getFPGATimestamp() > autoStopTime) {
        slowIntakeMotor(slowTime);
      }
    }

    if (intakeState == CoralManipulatorState.Slowing) {
      double preSpeed = intakeSpeed;
      setIntakeMotorSpeed(intakeSpeed - slowingFactor); // Slow motor as it approches stopping
      if (Math.abs(intakeSpeed) < 0.05 || (preSpeed > 0) ? intakeSpeed < 0 : intakeSpeed > 0) {
        stopIntakeMotor();
      }
    }
  }

  private enum CoralManipulatorState {
    Active,
    Auto,
    Slowing,
    Stopped
  }
}
