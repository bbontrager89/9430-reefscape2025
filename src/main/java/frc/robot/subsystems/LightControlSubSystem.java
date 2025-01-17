// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightControlSubSystem extends SubsystemBase {
  private double requestedStopTime;
  private boolean flickerModeOn;
  /** Creates a new LightControlSubSystem. */
  public LightControlSubSystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(flickerModeOn = false){
    //stop lights at rquested stop time
    if(Timer.getFPGATimestamp() > requestedStopTime){
      off();
    }
  }
  else{

  }
  }

  public void off(){
    //Turns off lights
  }
  public void on(){
    //turns on lights
  }
  public void turnOnFor(double timeOn){
    //set a time for lights to be on
    requestedStopTime = Timer.getFPGATimestamp() + timeOn;
    on();
    flickerModeOn = false;
  }
  public void flickerFor(double flickerOn){
    requestedStopTime = Timer.getFPGATimestamp() + flickerOn;
    flickerModeOn = true;
  }
}
