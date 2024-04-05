package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase{
    public final PWM ledPWM;
    private int pulseTime;
    private boolean hazardLightsOn;
    private boolean robotInMotion;

    public LedSubsystem(int id) { 
        ledPWM = new PWM(id);
        pulseTime = 0; // default pulse time
    }

    private void set(int value) {
        if (!hazardLightsOn) { // only change pulse time if hazard lights are not on
            pulseTime = value;
        }
        if (hazardLightsOn) {
            pulseTime = 10;
        }
    }

    public void setHazardLights() {
        hazardLightsOn = true;
        set(10);
    }

    public void cancelHazardLights() {
        hazardLightsOn = false;
        set(70);
    }

    public void setSpatulaAngle(double degrees) {
            if (degrees > 10 && degrees <= 340) {
                set(20);
            }
            else {
                if(pulseTime == 20) {
                    set(70);
                }
            }
        }

    public void setAutonEnabled() {
        set(30);
    }

    public void setClimberExtend() {
        set(40);
    }

    public void setClimberRetract() {
        set(50);
    }

    public void setRobotInMotion(double xspeed, double yspeed, double anglespeed) {
        if (Math.abs(xspeed) > 0.1 || Math.abs(yspeed) > 0.1 || Math.abs(anglespeed) > 0.1) {
            set(60);
        }
        else {
            if (pulseTime == 60) {
                set(70);
            }
        }        
    }

    public void setTeleopEnabled() {
        set(70);
    }

    public void setRobotDisabled() {
        set(0);
    }

    @Override
    public void periodic() {
        ledPWM.setPulseTimeMicroseconds(pulseTime);
        //System.out.print("PWM: " + pulseTime);
    }
}