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
  private LEDPattern solidPattern;
  

  /** Creates a new LightControlSubSystem. */
  public LightControlSubSystem() {
    lights.setLength(lightBuffer.getLength());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("light", isLightOn);
    //sets the pattern for the lights and updates it (in this case the pattern is always a solid color)
    solidPattern.applyTo(lightBuffer);
    lights.setData(lightBuffer);

    switch (lightStatus) {
      case OFF:
      //keeps the light off, this is the default state
      if(isLightOn){
        setOff();
      }
        break;

      case TIMEDSOLIDLIGHT:
      //turns the light on for a specified amount of time
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus = LightStatus.OFF;
      }
      if(!isLightOn){
      setOn();
      }
        break;

        case SOLIDLIGHT:
        //turns the light on without a timer
        if(!isLightOn){
        setOn();
        }
        break;


      case TIMEDFLICKER:
      //flickers the lights at a specified rate for a specified amount of time
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
         //sets how long the light should be off before switching
          flickerInterval = flickerIntervalLength + Timer.getFPGATimestamp();
          isFlickerTimerOn = true;
        } else if(!isLightOn && isFlickerTimerOn && flickerInterval < Timer.getFPGATimestamp()) {
          //switches the light state
          setOn();
          isFlickerTimerOn = false;
        }
        break;

        case FLICKER:
        //flickers the lights at a specified rate without a timer
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
         //sets how long the light should be off before switching
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
    //switches the state of the lights to off
    lightStatus = LightStatus.OFF;
  }

  public void turnOnFor( Color color, double timeOn) {
    //switches the state of the lights to on and sets a timer
    lightStatus = LightStatus.TIMEDSOLIDLIGHT;
    requestedStopTime = timeOn + Timer.getFPGATimestamp(); 
    //sets the color for the lights   
    solidPattern = LEDPattern.solid(color);
  }

  public void turnOn(Color color) {
    //switches the state of the lights to on
    lightStatus = LightStatus.SOLIDLIGHT;
    //sets the color for the lights
    solidPattern = LEDPattern.solid(color);
  }

  public void flickerFor(double flicker, Color color,double timeOn) {
    //switches the state of the lights to flicker and sets a timer
    lightStatus = LightStatus.TIMEDFLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
    //sets the flicker interval
    flickerIntervalLength = flicker;
    //sets the color for the lights
    solidPattern = LEDPattern.solid(color);
  }

  public void flicker(double flicker, Color color) {
    //switches the state of the lights to flicker without a timer
    lightStatus = LightStatus.FLICKER;
    //sets the flicker interval
    flickerIntervalLength = flicker;
    //sets the color for the lights
    solidPattern = LEDPattern.solid(color);
  }

  public void stateSet(Color color, LightStatus state, double timeOn) {
    //sets a timer
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
    //sets the color for the lights
    solidPattern = LEDPattern.solid(color);
    //switches light status
    lightStatus = state;
  }
  public void stateSet(Color color, LightStatus state) {
    //sets the color for the lights
    solidPattern = LEDPattern.solid(color);
    //switches light status
    lightStatus = state;
  }
}
