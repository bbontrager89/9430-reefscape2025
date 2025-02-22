// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeManipulatorSubsystem extends SubsystemBase {

  private SparkFlex rightAlgaeManipulatorMotor = new SparkFlex(AlgaeConstants.RightAlgaeManipulatorCANid, MotorType.kBrushless);
  private SparkFlex leftAlgaeManipulatorMotor = new SparkFlex(AlgaeConstants.LeftAlgaeManipulatorCANid, MotorType.kBrushless);


  /** Creates a new AlgaeManipulatorSubSystem. */
  public AlgaeManipulatorSubsystem() {
    
  }

  public void setSpeed(double speed) {
    rightAlgaeManipulatorMotor.set(speed);
    leftAlgaeManipulatorMotor.set(speed);
  }

  public void stop() {
    rightAlgaeManipulatorMotor.stopMotor();
    leftAlgaeManipulatorMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
