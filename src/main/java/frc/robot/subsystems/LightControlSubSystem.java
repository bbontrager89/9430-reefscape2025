// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightControlSubSystem extends SubsystemBase {

  public enum LightStatus{OFF, SOLIDLIGHT, FASTFLICKER, FLICKER, SLOWFLICKER}
  private double requestedStopTime;
  private boolean isFlickerTimerOn;
  private boolean isLightOn;
  private LightStatus lightStatus = LightStatus.OFF;

  /** Creates a new LightControlSubSystem. */
  public LightControlSubSystem() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("light", isLightOn);
    
switch (lightStatus) {
  case OFF :
  setOff();
  break;
  
  case SOLIDLIGHT :
  setOn();
  break;

  case FASTFLICKER :
  if (isLightOn) {
    setOff();
  } else {
    setOn();
  }
  break;
  case FLICKER :
  if (isLightOn == true && isFlickerTimerOn == false) {
    isFlickerTimerOn = true;
    new Thread(() ->  {
      try{
        Timer.delay(0.125);
      } catch (Exception e){
      }
    }); 
    isFlickerTimerOn = false;
    setOff();
  } else if (isLightOn == false && isFlickerTimerOn == false) {
    isFlickerTimerOn = true;
    new Thread(() ->  {
      try{
        Timer.delay(0.125);
      } catch (Exception e){
      }
    });
    isFlickerTimerOn = false;
    setOn();    
  }
  break;

  case SLOWFLICKER :
  if (isLightOn == true && isFlickerTimerOn == false) {
    isFlickerTimerOn = true;
    new Thread(() ->  {
      try{
        Timer.delay(0.25);
      } catch (Exception e){
      }
    }); 
    isFlickerTimerOn = false;
    setOff();
  } else if (isLightOn == false && isFlickerTimerOn == false) {
    isFlickerTimerOn = true;
    new Thread(() ->  {
      try{
        Timer.delay(0.25);
      } catch (Exception e){
      }
    });
    isFlickerTimerOn = false;
    setOn();    
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
    lightStatus = LightStatus.SOLIDLIGHT;
    new Thread(() ->  {
      try{
        Timer.delay(timeOn);
      } catch (Exception e){
      }
    }); lightStatus = LightStatus.OFF;
  }

  public void fastFlickerFor(double timeOn) {
    lightStatus = LightStatus.FASTFLICKER;
    new Thread(() ->  {
      try{
        Timer.delay(timeOn);
      } catch (Exception e){
      }
    }); lightStatus = LightStatus.OFF;
  }

  public void flickerFor(double timeOn) {
    lightStatus = LightStatus.FLICKER;
    new Thread(() ->  {
      try{
        Timer.delay(timeOn);
      } catch (Exception e){
      }
    }); lightStatus = LightStatus.OFF;
  }

  public void slowFlickerFor(double timeOn) {
    lightStatus = LightStatus.SLOWFLICKER;
    new Thread(() ->  {
      try{
        Timer.delay(timeOn);
      } catch (Exception e){
      }
    }); lightStatus = LightStatus.OFF;
  }
}
