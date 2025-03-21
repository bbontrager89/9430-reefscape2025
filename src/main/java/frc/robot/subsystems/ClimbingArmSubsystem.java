// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingArmConstants;

public class ClimbingArmSubsystem extends SubsystemBase {

  private SparkFlex climbingMotor = new SparkFlex(ClimbingArmConstants.climbingMotorCanId, MotorType.kBrushless);

  /** Creates a new ClimbingArmSubsystem. */
  public ClimbingArmSubsystem() {}

  /**
   * Sets the speed of the motors controlling the climbing arm
   * The speed will always be negative because the motor should always turn in that direction
   * 
   * @param speed the speed of the motors
   */
  public void setMotorSpeed(double speed) {
    climbingMotor.set(-Math.abs(speed)); // The climbing motor must always be **negative**
  }

  /**
   * Stops the climbing arm
   */
  public void stopMotor() {
    climbingMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
