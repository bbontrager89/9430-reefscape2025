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
  private PIDController scoringController;
  private PIDController transitController;

  private boolean doAutoCurrentLimit = true;
  private double autoStopTime = Double.POSITIVE_INFINITY;
  private double slowingFactor = 0.1;
  private double slowTime = 0.0;

  private double intakeSpeed = 0.0;
  private boolean isIntakeMotorOn = false;
  private boolean isPivotMotorOn = false;

  private boolean coralIntaken = false;

  private CoralManipulatorState intakeState = CoralManipulatorState.Stopped;

  private double intakeOnTimestamp = Double.NEGATIVE_INFINITY;
  private double currentLimitTimestamp = Double.NEGATIVE_INFINITY;

  private boolean abovePivotMaxHeight = false;
  private boolean belowPivotMinHeight = false;

  private double desiredPivotPosition = CoralManipulatorConstants.maximumPivotPosition;

  private static SendableChooser<Command> pivotCommands;

  private double pivotSpeed = 0.0;

  /** Creates a new CoralManipulatorSubsystem. */
  public CoralManipulatorSubsystem() {

    configureDashboardControls();

    pivotEncoder = pivotMotor.getAbsoluteEncoder();

    pivotController = new PIDController(CoralManipulatorConstants.pivotKp, CoralManipulatorConstants.pivotKi, CoralManipulatorConstants.pivotKd);
    pivotController.setTolerance(0.001);
    // pivotController.setIntegratorRange(0.13, 0.47);

    pivotController.reset();

    scoringController = new PIDController(CoralManipulatorConstants.scoringKp, CoralManipulatorConstants.scoringKi, CoralManipulatorConstants.scoringKd);
    scoringController.setTolerance(0.001);

    transitController = new PIDController(CoralManipulatorConstants.transitKp, CoralManipulatorConstants.transitKi, CoralManipulatorConstants.transitKd);
    transitController.setTolerance(0.001);
    
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
      stopPivotMotor(); 
      return;
    }
    if (!(abovePivotMaxHeight && (speed < 0)) && !(belowPivotMinHeight && (speed > 0))) {
     pivotMotor.set(speed);
     pivotSpeed = speed;
    } else {
      stopPivotMotor();
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

  public void movePivotTo(double pos) {
    pivotController.reset();
    scoringController.reset();

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

    if (intakeState == CoralManipulatorState.Stopped) {
      intakeOnTimestamp = Timer.getFPGATimestamp();
      isIntakeMotorOn = true;
      intakeState = CoralManipulatorState.Active;
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

  public boolean isCoralIntaken() {
    return coralIntaken;
  }

  public double intakeUptime() {
    return (isIntakeMotorOn) ? Timer.getFPGATimestamp() - intakeOnTimestamp : 0.0;
  }

  public double getDesiredPosition() {
    return desiredPivotPosition;
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

    // Clamp the speed of one of the PID controllers
    // PID controller changed based on desired height
    setPivotMotorSpeed(
      Math.min(
      Math.max(
        -((desiredPivotPosition == CoralManipulatorConstants.intakePivotPosition)
          ? pivotController 
          : (desiredPivotPosition == CoralManipulatorConstants.maximumPivotPosition)
          ? transitController 
          : scoringController)
            .calculate(getPivotMotorPosition(), desiredPivotPosition), 
      -CoralManipulatorConstants.maxPivotSpeed),
      CoralManipulatorConstants.maxPivotSpeed));
    

    // Check if motor is stuck to prevent over straining it
    if (doAutoCurrentLimit) {

      if (intakeMotor.getOutputCurrent() > CoralManipulatorConstants.autoStopCurrent && intakeSpeed < 0) {

        if (currentLimitTimestamp < 0) {
          currentLimitTimestamp = Timer.getFPGATimestamp();
        }

        if (currentLimitTimestamp + 0.2 < Timer.getFPGATimestamp()) {
          stopIntakeMotor();
          coralIntaken = true;
        }

      } else {
        currentLimitTimestamp = Double.NEGATIVE_INFINITY;
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
