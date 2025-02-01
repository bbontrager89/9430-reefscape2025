// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightControlSubSystem extends SubsystemBase {

  public enum LightStatus {
    OFF,
    TIMEDSOLIDLIGHT,
    SOLIDLIGHT,
    TIMEDFLICKER,
    FLICKER
  }

  private double requestedStopTime;
  private boolean isFlickerTimerOn = false;
  private double flickerInterval;
  private double flickerIntervalLength;
  private boolean isLightOn;
  private LightStatus lightStatus;
  private AddressableLED lights = new AddressableLED(Constants.LightConstants.PWMPort);
  private AddressableLEDBuffer lightBuffer = new AddressableLEDBuffer(Constants.LightConstants.lightLength);
  LEDPattern solidPattern;
  

  /** Creates a new LightControlSubSystem. */
  public LightControlSubSystem() {
    lights.setLength(lightBuffer.getLength());
    
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("light", isLightOn);
    solidPattern.applyTo(lightBuffer);
    lights.setData(lightBuffer);

    switch (lightStatus) {
      case OFF:
      if(isLightOn){
        setOff();
      }
        break;

      case TIMEDSOLIDLIGHT:
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus = LightStatus.OFF;
      }
      if(!isLightOn){
      setOn();
      }
        break;

        case SOLIDLIGHT:
        if(!isLightOn){
        setOn();
        }
        break;


      case TIMEDFLICKER:
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus = LightStatus.OFF;
      }
        if (isLightOn && !isFlickerTimerOn) {
          //sets how long the light should be on before switching
          flickerInterval = flickerIntervalLength + Timer.getFPGATimestamp();   
          isFlickerTimerOn = true;
        } else if (isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()){
          //switches the light state
          setOff();
          isFlickerTimerOn = false;
        }
        if(!isLightOn && !isFlickerTimerOn){
         //sets how long the light should be on before switching
          flickerInterval = flickerIntervalLength + Timer.getFPGATimestamp();
          isFlickerTimerOn = true;
        } else if(!isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()) {
          //switches the light state
          setOn();
          isFlickerTimerOn = false;
        }
        break;

        case FLICKER:
        if (isLightOn && !isFlickerTimerOn) {
          //sets how long the light should be on before switching
          flickerInterval = flickerIntervalLength + Timer.getFPGATimestamp();   
          isFlickerTimerOn = true;
        } else if (isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()){
          //switches the light state
          setOff();
          isFlickerTimerOn = false;
        }
        if(!isLightOn && !isFlickerTimerOn){
         //sets how long the light should be on before switching
          flickerInterval = flickerIntervalLength + Timer.getFPGATimestamp();
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
    lights.stop();
    isLightOn = false;
  }

  public void setOn() {
    // turns on lights
    lights.start();
    isLightOn = true;
  }

  public void turnOff(){
    //switches the state of the lights
    lightStatus = LightStatus.OFF;
  }

  public void turnOnFor( Color color, double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.TIMEDSOLIDLIGHT;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();    
    solidPattern = LEDPattern.solid(color);
  }

  public void turnOn(Color color) {
    //switches the state of the lights
    lightStatus = LightStatus.SOLIDLIGHT;
    solidPattern = LEDPattern.solid(color);
  }

  public void flickerFor(double flicker, Color color,double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.TIMEDFLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
    flickerIntervalLength = flicker;
    solidPattern = LEDPattern.solid(color);
  }

  public void flicker(double flicker, Color color) {
    //switches the state of the lights
    lightStatus = LightStatus.FLICKER;
    flickerIntervalLength = flicker;
    solidPattern = LEDPattern.solid(color);
  }

  public void stateSet(Color color, LightStatus state, double timeOn) {
    
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
    
    solidPattern = LEDPattern.solid(color);
    lightStatus = state;
  }
  public void stateSet(Color color, LightStatus state) {
    solidPattern = LEDPattern.solid(color);
    lightStatus = state;
  }
}
