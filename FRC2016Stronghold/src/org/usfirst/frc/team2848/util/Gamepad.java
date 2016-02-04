package org.usfirst.frc.team2848.util;

import edu.wpi.first.wpilibj.Joystick;


public class Gamepad {
    private Joystick joystick;
    
    public Gamepad(int portNum) {
        this.joystick = new Joystick(portNum);
    }
    
    public double getLeftX() {
        return this.joystick.getRawAxis(0);
    }
    
    public double getLeftY() {
        return -this.joystick.getRawAxis(1);
    }
    
    public double getTriggerLeft() {
    	return this.joystick.getRawAxis(2);
    	
    }
    
    public double getTriggerRight(){
    	return this.joystick.getRawAxis(3);
    }
    
    public double getRightX() {
        return this.joystick.getRawAxis(4);
    }
    
    public double getRightY() {
        return -this.joystick.getRawAxis(5);
    }
    
    public boolean getButton(int btnNum) {
        return this.joystick.getRawButton(btnNum);
    }
    
    public boolean getGreenButton() {
        return this.joystick.getRawButton(1);
    }
    
    public boolean getBlueButton() {
        return this.joystick.getRawButton(3);
    }
    
    public boolean getRedButton() {
        return this.joystick.getRawButton(2);
    }
    
    public boolean getYellowButton() {
        return this.joystick.getRawButton(4);
    }
    
    public boolean getBackButton() {
        return this.joystick.getRawButton(7);
    }
    
    public boolean getStartButton() {
        return this.joystick.getRawButton(8);
    }
    
    public boolean getLeftBumper() {
        return this.joystick.getRawButton(5);
    }
    
    public boolean getRightBumper() {
        return this.joystick.getRawButton(6);
    }
    
    public boolean getLeftStickClick() {
        return this.joystick.getRawButton(9);
    }
    
    public boolean getRightStickClick() {
        return this.joystick.getRawButton(10);
    }
    
   
	
	
    
}
