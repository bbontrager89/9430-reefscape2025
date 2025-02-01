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
    TIMEDFASTFLICKER,
    FASTFLICKER,
    TIMEDFLICKER,
    FLICKER,
    TIMEDSLOWFLICKER,
    SLOWFLICKER
  }

  private double requestedStopTime;
  private boolean isFlickerTimerOn = false;
  private double flickerInterval;
  private boolean isLightOn;
  private LightStatus lightStatus;
  private AddressableLED lights = new AddressableLED(Constants.LightConstants.PWMPort);
  private AddressableLEDBuffer lightBuffer = new AddressableLEDBuffer(Constants.LightConstants.lightLength);
  

  /** Creates a new LightControlSubSystem. */
  public LightControlSubSystem() {
    lights.setLength(lightBuffer.getLength());
    LEDPattern solidPattern = LEDPattern.solid(Color.kWhite);
    solidPattern.applyTo(lightBuffer);
    lights.setData(lightBuffer);
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

      case TIMEDFASTFLICKER:
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

        case FASTFLICKER:
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

      case TIMEDFLICKER:
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus = LightStatus.OFF;
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

        case FLICKER:
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

      case TIMEDSLOWFLICKER:
      if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus = LightStatus.OFF;
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

        case SLOWFLICKER:
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

  public void turnOnFor(double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.TIMEDSOLIDLIGHT;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();    
  }

  public void turnOn() {
    //switches the state of the lights
    lightStatus = LightStatus.SOLIDLIGHT;
  }

  public void fastFlickerFor(double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.TIMEDFASTFLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
  }

  public void fastFlicker() {
    //switches the state of the lights
    lightStatus = LightStatus.FASTFLICKER;
  }

  public void flickerFor(double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.TIMEDFLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
  }

  public void flicker() {
    //switches the state of the lights
    lightStatus = LightStatus.FLICKER;
  }

  public void slowFlickerFor(double timeOn) {
    //switches the state of the lights and sets a timer
    lightStatus = LightStatus.TIMEDSLOWFLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
  }

  public void slowFlicker() {
   //switches the state of the lights
    lightStatus = LightStatus.SLOWFLICKER;
  }
}
