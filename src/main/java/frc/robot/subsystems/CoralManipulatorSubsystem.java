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

public class CoralManipulatorSubsystem extends SubsystemBase {

  private SparkMax pivotMotor = new SparkMax(CoralManipulatorConstants.PivotMotorCanId, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(CoralManipulatorConstants.IntakeMotorCanId, MotorType.kBrushless);

  private SparkAbsoluteEncoder pivotEncoder;

  private PIDController pivotController;

  private CoralIntakeState intakeState = CoralIntakeState.Stopped;

  private boolean doCoralIntakenCheck = true;
  private double intakeAutoStopTime = Double.POSITIVE_INFINITY;
  private double intakeSlowingFactor = 0.1;
  private double intakeSlowingTime = 0.0;

  private double intakeSpeed = 0.0;
  private boolean isIntakeMotorOn = false;
  private boolean isPivotMotorOn = false;

  private boolean coralIntaken = false;

  private double intakeOnTimestamp = Double.NEGATIVE_INFINITY;
  private double intakeCurrentLimitTimestamp = Double.NEGATIVE_INFINITY;



  private boolean abovePivotMaxHeight = false;
  private boolean belowPivotMinHeight = false;

  private double desiredPivotPosition = 0.25;

  private double pivotSpeed = 0.0;

  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {

    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotController = new PIDController(CoralManipulatorConstants.pivotKp, CoralManipulatorConstants.pivotKi, CoralManipulatorConstants.pivotKd);
    pivotController.reset();

    configureDashboardControls();

    pivotController.setTolerance(0.001);
    pivotController.setIntegratorRange(0.13, 0.47);
  }

  private static SendableChooser<Command> pivotCommands;
  /**
   * Configures a sendable chooser for dashboard controls
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
      movePivotTo(0.48);
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
    if (speed == 0) {
      stopIntakeMotor(); 
      return;
    }
    if (!(abovePivotMaxHeight && (speed > 0)) && !(belowPivotMinHeight && (speed < 0))) {
     pivotMotor.set(speed);
     pivotSpeed = speed;
    }
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
    pivotSpeed = 0.0;
    isPivotMotorOn = false;
  }

  /**
   * Sets the desired pivot height to given position
   * 
   * @param pos the height relative to the absolute encoder
   */
  public void movePivotTo(double pos) {
    pivotController.reset();

    desiredPivotPosition = pos;
    pivotController = new PIDController(CoralManipulatorConstants.pivotKp, CoralManipulatorConstants.pivotKi, CoralManipulatorConstants.pivotKd);
    
    if (desiredPivotPosition > CoralManipulatorConstants.maximumPivotPosition) {
      desiredPivotPosition = CoralManipulatorConstants.maximumPivotPosition;
    }
    if (desiredPivotPosition < CoralManipulatorConstants.minimumPivotPosition) {
      desiredPivotPosition = CoralManipulatorConstants.minimumPivotPosition;
    }
    SmartDashboard.putNumber("Desired Pivot Height", desiredPivotPosition);
  }

  /**
   * Private method to set the intake motor speed.
   * Accessed by methods inside the subsystem.
   * 
   * @param speed the speed to set the motor
   */
  private void setIntakeMotorSpeed(double speed) {
    if (!(coralIntaken && speed < 0)) {
      intakeMotor.set(speed);
    }

    if (speed > 0) {
      coralIntaken = false;
    }

    intakeSpeed = speed;

    if (intakeState == CoralIntakeState.Stopped) {
      intakeOnTimestamp = Timer.getFPGATimestamp();
      isIntakeMotorOn = true;
      intakeState = CoralIntakeState.Active;
    }
  }

  /**
   * Public method to set the intake motor speed.
   * Also sets the state of the motor to Active
   * 
   * @param speed the speed to set the motor
   */
  public void startIntakeMotor(double speed) {
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
    intakeAutoStopTime = time + Timer.getFPGATimestamp();
    setIntakeMotorSpeed(speed);
    intakeState = CoralIntakeState.Auto;
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
    intakeAutoStopTime = time + Timer.getFPGATimestamp();
    intakeSlowingTime = slowingTime;
    setIntakeMotorSpeed(speed);
    intakeState = CoralIntakeState.Auto;
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
    intakeSlowingFactor = (intakeSpeed / (50 * time));
    intakeState = CoralIntakeState.Slowing;
  }

  /**
   * Stops the intake motor immediatly.
   * Sets the intake state to stopped
   */
  public void stopIntakeMotor() {
    intakeMotor.stopMotor();
    intakeState = CoralIntakeState.Stopped;

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

  /**
   * Returns if pivot is within tollerance
   * 
   * @return boolean
   */
  public boolean atPivotPosition() {
    return pivotController.atSetpoint();
  }

  /**
   * Returns if coral is perdicted to have been intaken
   * if the output current of the intake motor is above
   * a threshold for an amount of time than coral is assumed
   * to have been intaken until the motors eject.
   *  
   * @return boolean 
   */
  public boolean isCoralIntaken() {
    return coralIntaken;
  }

  /**
   * Returns the amount of time the coral intake motor has been running
   * 
   * @return double representing time in seconds
   */
  public double intakeUptime() {
    return (isIntakeMotorOn) ? Timer.getFPGATimestamp() - intakeOnTimestamp : 0.0;
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
    SmartDashboard.putNumber("Intake Motor Stop Time", intakeAutoStopTime);
    SmartDashboard.putNumber("Intake Motor Timestamp", intakeOnTimestamp);
    SmartDashboard.putNumber("Intake Motor Speed", intakeSpeed);
    SmartDashboard.putNumber("Pivot Error", pivotController.getError());
    SmartDashboard.putNumber("desiredPivotPosition", desiredPivotPosition);
    SmartDashboard.putBoolean("Coral Intaken", coralIntaken);
    SmartDashboard.putNumber("Intake Current", intakeMotor.getOutputCurrent());

    if (getPivotMotorPosition() > CoralManipulatorConstants.maximumPivotPosition) {
      abovePivotMaxHeight = true;
      if (pivotSpeed > 0) {
        stopPivotMotor();
      }
    } else {
      abovePivotMaxHeight = false;
    }

    if (getPivotMotorPosition() < CoralManipulatorConstants.minimumPivotPosition) {
      belowPivotMinHeight = true;
      if (pivotSpeed < 0) {
        stopPivotMotor();
      }
    } else {
      belowPivotMinHeight = false;
    }

    pivotMotor.set(Math.min(Math.max(-pivotController.calculate(getPivotMotorPosition(), desiredPivotPosition), -CoralManipulatorConstants.maxPivotSpeed),CoralManipulatorConstants.maxPivotSpeed));
    

    // Check if motor is stuck to prevent over straining it
    if (doCoralIntakenCheck) {

      if (intakeMotor.getOutputCurrent() > CoralManipulatorConstants.autoStopCurrent && intakeSpeed < 0) {

        if (intakeCurrentLimitTimestamp < 0) {
          intakeCurrentLimitTimestamp = Timer.getFPGATimestamp();
        }

        if (intakeCurrentLimitTimestamp + 0.2 < Timer.getFPGATimestamp()) {
          stopIntakeMotor();
          coralIntaken = true;
        }

      } else {
        intakeCurrentLimitTimestamp = Double.NEGATIVE_INFINITY;
      }

    }

    // Turn off motor after time has passed
    if (intakeState == CoralIntakeState.Auto) {
      if (Timer.getFPGATimestamp() > intakeAutoStopTime) {
        slowIntakeMotor(intakeSlowingTime);
      }
    }

    if (intakeState == CoralIntakeState.Slowing) {
      double preSpeed = intakeSpeed;
      setIntakeMotorSpeed(intakeSpeed - intakeSlowingFactor); // Slow motor as it approches stopping
      if (Math.abs(intakeSpeed) < 0.05 || (preSpeed > 0) ? intakeSpeed < 0 : intakeSpeed > 0) {
        stopIntakeMotor();
      }
    }
  }

  /**
   * Enum for the state the Coral Intake motor is in
   */
  private enum CoralIntakeState {
    Active,
    Auto,
    Slowing,
    Stopped
  }
}
