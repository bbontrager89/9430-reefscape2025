// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightControlSubSystem extends SubsystemBase {
  private double requestedStopTime;
  private boolean flickerModeOn;
  private boolean isLightOn;
  /** Creates a new LightControlSubSystem. */
  public LightControlSubSystem() {}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("flickerMode", flickerModeOn);
    // This method will be called once per scheduler run
    if(!flickerModeOn){
    //stop lights at rquested stop time
    if(Timer.getFPGATimestamp() > requestedStopTime){
      off();
    }
  }
  else{
    //flicker lights until requested stop time is reached
    if (!isLightOn){
      on();
    }
    else {
      off();
    }
    if(Timer.getFPGATimestamp() > requestedStopTime){
      off();
      flickerModeOn = false;
    }
  }
  }

  public void off(){
    //Turns off lights
    SmartDashboard.putBoolean("light", false);

    isLightOn = false;
  }
  public void on(){
    //turns on lights
    SmartDashboard.putBoolean("light", true);

    isLightOn = true;
  }
  public void turnOnFor(double timeOn){
    //set a time for lights to be on
    requestedStopTime = Timer.getFPGATimestamp() + timeOn;
    on();
    flickerModeOn = false;
  }
  public void flickerFor(double flickerOn){
    //set time for how long lights should flicker for
    requestedStopTime = Timer.getFPGATimestamp() + flickerOn;
    //set that lights will flicker
    flickerModeOn = true;
  }
}
