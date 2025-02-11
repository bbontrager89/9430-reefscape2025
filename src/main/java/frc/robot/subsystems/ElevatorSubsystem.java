// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.Elastic;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkFlex elevatorMotor = new SparkFlex(ElevatorConstants.elevatorMotorCanId, MotorType.kBrushless);
  private AbsoluteEncoder absoluteEncoder = elevatorMotor.getAbsoluteEncoder();

  private SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();

  private double desiredHeight;
  private boolean autoMode = false;

  private boolean aboveMaxHeight = false;
  private boolean belowMinHeight = false;
  private double currentSpeed = 0.0;

  private static SendableChooser<Command> elevatorCommands;

  // Create a WPILib PIDController using your constants.
  // If you only have kP defined, set kI and kD to 0.
  private final PIDController elevatorPIDController = new PIDController(
      ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {

    elevatorMotorConfig.inverted(ElevatorConstants.elevatorMotorInverted);
    elevatorMotor.configure(elevatorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Configure the PID tolerance (how close in position you need to be to the setpoint).
    elevatorPIDController.setTolerance(ElevatorConstants.positionTolerence);

    configureDashboardControls();
  }

  public void configureDashboardControls() {

    elevatorCommands = new SendableChooser<Command>();

    elevatorCommands.setDefaultOption("SP 1", new InstantCommand(() -> {
      moveToScoringPosition(1);
    }));
    elevatorCommands.addOption("SP 2", new InstantCommand(() -> {
      moveToScoringPosition(2);
    }));
    elevatorCommands.addOption("SP 3", new InstantCommand(() -> {
      moveToScoringPosition(3);
    }));
    elevatorCommands.addOption("min", new InstantCommand(() -> {
      moveToScoringPosition(4);
    }));
    elevatorCommands.addOption("max", new InstantCommand(() -> {
      moveToScoringPosition(5);
    }));
    elevatorCommands.addOption("Custom", new InstantCommand(() -> {
      moveToPosition(SmartDashboard.getNumber("Custom Elevator Height", ElevatorConstants.level1ScoringPosition));
    }));

    SmartDashboard.putData("Elevator Height Commands", elevatorCommands);

    SmartDashboard.putData("Run Elevator Command", new ScheduleCommand(new InstantCommand(() -> {
      if (elevatorCommands.getSelected() != null)
        elevatorCommands.getSelected().schedule();
    })));

    SmartDashboard.putNumber("Custom Elevator Height", ElevatorConstants.level1ScoringPosition);
    SmartDashboard.putNumber("Desired Height", 0.0);
  }

  public void setMotorSpeed(double speed) {
    currentSpeed = speed;
    // Check soft limits before commanding the motor.
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
        Elastic.sendError("Invalid Scoring Position", "requested in ElevatorSubsystem");
        return;
    }

    SmartDashboard.putNumber("Desired Height", desiredHeight);

    // Set the PID setpoint and enable auto mode.
    elevatorPIDController.setSetpoint(desiredHeight);
    autoMode = true;
  }

  public void moveToPosition(double position) {
    desiredHeight = position;
    SmartDashboard.putNumber("Desired Height", desiredHeight);

    // Set the PID setpoint and enable auto mode.
    elevatorPIDController.setSetpoint(desiredHeight);
    autoMode = true;
  }

  public void turnOffAutoMode() {
    autoMode = false;
    stopMotor();
  }

  public double getHeight() {
    return absoluteEncoder.getPosition();
  }

  public boolean atHeight() {
    return (Math.abs(getHeight() - desiredHeight) < ElevatorConstants.positionTolerence);
  }

  @Override
  public void periodic() {
    // Log current sensor data.
    double height = getHeight();
    SmartDashboard.putNumber("Elevator Position", height);
    SmartDashboard.putNumber("Elevator Velocity", absoluteEncoder.getVelocity());

    // If in auto mode, compute the PID output.
    double pidOutput = 0.0;
    if (autoMode) {
      pidOutput = elevatorPIDController.calculate(height, elevatorPIDController.getSetpoint());
      pidOutput = MathUtil.clamp(pidOutput, -ElevatorConstants.maximumAutoSpeed, ElevatorConstants.maximumAutoSpeed);
      SmartDashboard.putNumber("AutoSpeed", pidOutput);
    }

    // Soft limit checks

    // Lower soft limit check (when elevator is too high).
    if (height > ElevatorConstants.maximumElevatorHeight) {
      // If the motor is moving downward (negative speed) while above maximum,
      // stop the motor.
      if (currentSpeed < 0) {
        stopMotor();
      }
      // If auto mode is active and the PID output would drive the elevator down further,
      // disable auto mode.
      if (autoMode && pidOutput < 0) {
        turnOffAutoMode();
      }
      aboveMaxHeight = true;
    } else {
      aboveMaxHeight = false;
    }

    // Upper soft limit check (when elevator is too low).
    if (height < ElevatorConstants.minimumElevatorHeight) {
      // If the motor is moving upward (positive speed) while below minimum,
      // stop the motor.
      if (currentSpeed > 0) {
        stopMotor();
      }
      // If auto mode is active and the PID output would drive the elevator upward,
      // disable auto mode.
      if (autoMode && pidOutput > 0) {
        turnOffAutoMode();
      }
      belowMinHeight = true;
    } else {
      belowMinHeight = false;
    }

    // If auto mode is still enabled, command the motor using the PID controller’s output.
    if (autoMode) {
      // If the controller is within tolerance of the setpoint, stop auto mode.
      if (elevatorPIDController.atSetpoint()) {
        turnOffAutoMode();
      } else {
        setMotorSpeed(pidOutput);
      }
    }
  }
}
