// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightControlSubSystem extends SubsystemBase {

  public enum LightStatus {
    OFF,
    SOLIDLIGHT,
    FASTFLICKER,
    FLICKER,
    SLOWFLICKER
  }

  private double requestedStopTime;
  private boolean isFlickerTimerOn = false;
  private double flickerInterval;
  private boolean isLightOn;
  private LightStatus lightStatus = LightStatus.OFF;

  /** Creates a new LightControlSubSystem. */
  public LightControlSubSystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("light", isLightOn);

    switch (lightStatus) {
      case OFF:
      if(isLightOn){
        setOff();
      }
        break;

      case SOLIDLIGHT:
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus =LightStatus.OFF;
      }
      if(!isLightOn){
      setOn();
      }
        break;

      case FASTFLICKER:
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus = LightStatus.OFF;
      }
      if (isLightOn && !isFlickerTimerOn) {
        //sets how long the light should be on before switching
        flickerInterval = 0.125 + Timer.getFPGATimestamp();   
        isFlickerTimerOn = true;
      } else if (isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()){
        //switches the light state
        setOff();
        isFlickerTimerOn = false;
      }
      if(!isLightOn && !isFlickerTimerOn){
        //sets how long the light should be on before switching
        flickerInterval = 0.125 + Timer.getFPGATimestamp();
        isFlickerTimerOn = true;
      } else if(!isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()) {
        setOn();
        isFlickerTimerOn = false;
      }
        break;
      case FLICKER:
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus =LightStatus.OFF;
      }
        if (isLightOn && !isFlickerTimerOn) {
          //sets how long the light should be on before switching
          flickerInterval = 0.25 + Timer.getFPGATimestamp();   
          isFlickerTimerOn = true;
        } else if (isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()){
          //switches the light state
          setOff();
          isFlickerTimerOn = false;
        }
        if(!isLightOn && !isFlickerTimerOn){
         //sets how long the light should be on before switching
          flickerInterval = 0.25 + Timer.getFPGATimestamp();
          isFlickerTimerOn = true;
        } else if(!isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()) {
          //switches the light state
          setOn();
          isFlickerTimerOn = false;
        }
        break;

      case SLOWFLICKER:
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus =LightStatus.OFF;
      }
        if (isLightOn && !isFlickerTimerOn) {
          //sets how long the light should be on before switching
          flickerInterval = 0.5 + Timer.getFPGATimestamp();   
          isFlickerTimerOn = true;
        } else if (isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()){
          //switches the light state
          setOff();
          isFlickerTimerOn = false;
        }
        if(!isLightOn && !isFlickerTimerOn){
          //sets how long the light should be on before switching
          flickerInterval = 0.5 + Timer.getFPGATimestamp();
          isFlickerTimerOn = true;
        } else if(!isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()) {
          //switches the light state
          setOn();
          isFlickerTimerOn = false;
        }
        break;
    }
  }

  public void setOff() {
    // Turns off lights
    isLightOn = false;
  }

  public void setOn() {
    // turns on lights
    isLightOn = true;
  }

  public void turnOnFor(double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.SOLIDLIGHT;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();    
  }

  public void fastFlickerFor(double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.FASTFLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
  }

  public void flickerFor(double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.FLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
  }

  public void slowFlickerFor(double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.SLOWFLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
  }
}
