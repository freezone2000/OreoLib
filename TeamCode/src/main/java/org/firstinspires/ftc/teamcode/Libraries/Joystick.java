package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by kk200 on 9/29/2017.
 */

public class Joystick{
    public Gamepad gamepad;

    public Joystick(Gamepad joy){
        this.gamepad = joy;
    }

    public void set(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean buttonA() {
        return gamepad.a && !secondFunc();
    }

    public boolean buttonB() {
        return gamepad.b && !secondFunc();
    }

    public boolean buttonX() {
        return gamepad.x && !secondFunc();
    }

    public boolean buttonY() {
        return gamepad.y && !secondFunc();
    }


    public boolean buttonUp() {
        return gamepad.dpad_up && !secondFunc();
    }

    public boolean buttonDown() {
        return gamepad.dpad_down && !secondFunc();
    }

    public boolean buttonRight() {
        return gamepad.dpad_right && !secondFunc();
    }

    public boolean buttonLeft() {
        return gamepad.dpad_left && !secondFunc();
    }

    public boolean leftBumper() {
        return gamepad.left_bumper; //Dont check for second func here because leftBump is second func
    }

    public boolean rightBumper() {
        return gamepad.right_bumper && !secondFunc();
    }

    public double leftTrigger(){
        double preReading = gamepad.left_trigger;
        float reading = (float)(Math.pow(gamepad.left_trigger,2));
        if(preReading < 0){
            reading*= -1;
        }
        return reading;
    }

    public double rightTrigger(){
        double preReading = gamepad.right_trigger;
        float reading = (float)(Math.pow(gamepad.right_trigger,2));
        if(preReading < 0){
            reading*= -1;
        }
        return reading;
    }

    public boolean buttonStart() {
        return gamepad.start;
    }

    @Deprecated
    public boolean buttonBack() {
        return gamepad.back;
    } //Doesnt work with Logitech Controller


    public float leftX(){
        double preReading = gamepad.left_stick_x;
        float reading = (float)(Math.pow(gamepad.left_stick_x,2));
        if(preReading < 0){
            reading*= -1;
        }
        return reading;
    }

    public float rightX(){
        double preReading = gamepad.right_stick_x;
        float reading = (float)(Math.pow(gamepad.right_stick_x,2));
        if(preReading < 0){
            reading*= -1;
        }
        return reading;
    }

    public float leftY(){
        double preReading = gamepad.left_stick_y;
        float reading = (float)(Math.pow(gamepad.left_stick_y,2));
        if(preReading < 0){
            reading*= -1;
        }
        return reading;
    }

    public float rightY(){
        double preReading = gamepad.right_stick_y;
        float reading = (float)(Math.pow(gamepad.right_stick_y,2));
        if(preReading < 0){
            reading*= -1;
        }
        return reading;
    }

    public boolean leftStickButton(){
        return gamepad.left_stick_button;
    }
    public boolean rightStickButton(){
        return gamepad.right_stick_button;
    }
    public boolean isAtRest(){
        return gamepad.atRest();
    }

    @Deprecated
    public boolean guide(){
        return gamepad.guide;
    }//Doesnt work with Logitec Controller

    //----------------------SECOND FUNCTION----------------------------------------------
    public boolean secondFunc() {
        return leftBumper();
    }
    public boolean secondButtonA(){
        return secondFunc() && buttonA();
    }
    public boolean secondButtonB(){
        return secondFunc() && buttonB();
    }
    public boolean secondButtonX(){
        return secondFunc() && buttonX();
    }
    public boolean secondButtonY(){
        return secondFunc() && buttonY();
    }
    public boolean secondButtonUp(){
        return secondFunc() && buttonUp();
    }
    public boolean secondButtonDown(){
        return secondFunc() && buttonDown();
    }
    public boolean secondButtonLeft(){
        return secondFunc() && buttonLeft();
    }
    public boolean secondButtonRight(){
        return secondFunc() && buttonRight();
    }
    public boolean secondRightBumper(){
        return secondFunc() && rightBumper();
    }
}