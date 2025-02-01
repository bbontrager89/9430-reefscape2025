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
        if (isLightOn) {
          setOff();
        } else {
          setOn();
        }
        break;
      case FLICKER:
      //still working on this
      /*if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus =LightStatus.OFF;
      }
        if (isLightOn) {
          flickerInterval = 0.25 + requestedStopTime;   
        } else {
          isFlickerTimerOn = true;
          new Thread(() -> {
            try {
              Timer.delay(0.25);
              isFlickerTimerOn = false;
              setOn();
            } catch (Exception e) {
              e.printStackTrace();
            }
          });
          
        }*/
        break;

      case SLOWFLICKER:
      //still working on this
      /*if(requestedStopTime < Timer.getFPGATimestamp())  {
        lightStatus =LightStatus.OFF;
      }
        if (isLightOn == true && isFlickerTimerOn == false) {
          isFlickerTimerOn = true;
          new Thread(() -> {
            try {
              Timer.delay(0.5);
              isFlickerTimerOn = false;
          setOff();
            } catch (Exception e) {
              e.printStackTrace();
            }
          });
          
        } else if (isLightOn == false && isFlickerTimerOn == false) {
          isFlickerTimerOn = true;
          new Thread(() -> {
            try {
              Timer.delay(0.5);
              isFlickerTimerOn = false;
              setOn();
            } catch (Exception e) {
              e.printStackTrace();
            }
          });
          
        }*/
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
    requestedStopTime = timeOn + Timer.getFPGATimestamp();    
  }

  public void fastFlickerFor(double timeOn) {
    lightStatus = LightStatus.FASTFLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
  }

  public void flickerFor(double timeOn) {
    lightStatus = LightStatus.FLICKER;
    requestedStopTime = timeOn + Timer.getFPGATimestamp();
  }

  public void slowFlickerFor(double timeOn) {
    lightStatus = LightStatus.SLOWFLICKER;
    new Thread(() -> {
      try {
        Timer.delay(timeOn);
        lightStatus = LightStatus.OFF;
      } catch (Exception e) {
        e.printStackTrace();
      }
    });
    
  }
}
