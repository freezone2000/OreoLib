package org.firstinspires.ftc.teamcode.Drivetrains;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libraries.IMUHelper;
import org.firstinspires.ftc.teamcode.Libraries.Joystick;

import java.util.HashMap;

/**
 * Created by kk200 on 10/3/2017.
 */

public abstract class Drivetrain {
    public volatile LinearOpMode op;
    public volatile Joystick joy1; //1st controller
    public volatile Joystick joy2; //2nd controller
    public volatile ElapsedTime timer;


    public volatile HashMap<String,DcMotor> motors = new HashMap<>(); //contains all motors on robot
    public volatile HashMap<String,DcMotor> driveMotors = new HashMap<>(); //contains motors that control wheels
    public volatile HashMap<String, Servo> servos = new HashMap<>(); //contains all servos on robot
    public volatile HashMap<String, ColorSensor> colorSensors = new HashMap<>(); //controls all color sensors on robot
    public volatile HashMap<String, IMUHelper> gyros = new HashMap<>(); //contains all gyros on robot
    public volatile HashMap<String,DcMotor> encoderDriveMotors = new HashMap<>(); //contains all motors with encoders
    public volatile HashMap<String, Thread> threads = new HashMap<>(); //contains all threads
    public volatile HashMap<String, DigitalChannel> touchsensors = new HashMap<>(); //contains all touch sensors

    /*
     * assigns default objects to some variables
     */
    Drivetrain(LinearOpMode op){
        this.op = op;
        this.joy1 = new Joystick(op.gamepad1); //assigns joy1 to the first joystick on the controller
        this.joy2 = new Joystick(op.gamepad2); //assigns joy2 to the secondary joystick on the controller
        this.timer = new ElapsedTime(); //assigns timer

    }

    public void resetTimer(){
        timer.reset();
    }

    public double getElapsedTime(){
        return timer.time();
    }

    /*
     * creates a new thread from the object passed in
     */
    public void addThread(Thread threadObject){
        if (threadObject != null) {
            try {
                this.threads.put(threadObject.getName(), threadObject);
            } catch (Exception e) {
                op.telemetry.addData("ERROR WHILE ADDING THREAD:", e);
            }
        }
    }

    /*
      adds gyro to list and logs to telemetry, unless error
     */
    public void addGyro(String gyroName){
        try {
            this.gyros.put(gyroName, new IMUHelper(op, op.hardwareMap.get(BNO055IMU.class, gyroName)));
            op.telemetry.addData("Debugging GYRO","Successfully added IMU: " + gyroName);
            op.telemetry.update();
        } catch (Exception e){
            op.telemetry.addData("ERROR WHILE ADDING GYRO:",e);
            op.telemetry.update();

        }
    }

    /*
     */
    public void addMotor(String motorName){
        try {
            this.motors.put(motorName, op.hardwareMap.dcMotor.get(motorName));
            op.hardwareMap.dcMotor.get(motorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e){
            op.telemetry.addData("ERROR WHILE ADDING MOTOR:",e);
            op.telemetry.update();

        }
    }

    public void addDriveMotor(String motorName){
        try {
            this.motors.put(motorName, op.hardwareMap.dcMotor.get(motorName));
            this.driveMotors.put(motorName,op.hardwareMap.dcMotor.get(motorName));
            op.hardwareMap.dcMotor.get(motorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e){
            op.telemetry.addData("ERROR WHILE ADDING DRIVE MOTOR:",e);
            op.telemetry.update();

        }
    }

    public void addEncoderDriveMotor(String motorName){
        try{
            this.motors.put(motorName,op.hardwareMap.dcMotor.get(motorName));
            this.driveMotors.put(motorName,op.hardwareMap.dcMotor.get(motorName));
            this.encoderDriveMotors.put(motorName,op.hardwareMap.dcMotor.get(motorName));
            op.hardwareMap.dcMotor.get(motorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e){
            op.telemetry.addData("ERROR WHILE ADDING ENCODER DRIVE MOTOR",e);
            op.telemetry.update();

        }
    }

    private int counter = 0;
    public void addServo(String servoName){
        try{
            this.servos.put(servoName, op.hardwareMap.servo.get(servoName));
            op.telemetry.addData("Debugging" + counter,"Successfully added servo: " + servoName);
            counter++;
            op.telemetry.update();
        } catch (Exception e){
            op.telemetry.addData("ERROR WHILE ADDING SERVO:",e);
            op.telemetry.update();
        }
    }

    public void addColorSensor(String sensorName){
        try{
            this.colorSensors.put(sensorName,op.hardwareMap.colorSensor.get(sensorName));
        } catch (Exception e){
            op.telemetry.addData("ERROR WHILE ADDING COLOR SENSOR:",e);
            op.telemetry.update();
        }
    }
    public void addTouchSensor(String sensorName){
        try{
            // get a reference to our digitalTouch object.
            DigitalChannel digitalTouch;
            digitalTouch = op.hardwareMap.get(DigitalChannel.class, sensorName);

            // set the digital channel to input.
            digitalTouch.setMode(DigitalChannel.Mode.INPUT);
            this.touchsensors.put(sensorName,digitalTouch);
        } catch (Exception e){
            op.telemetry.addData("ERROR WHILE ADDING TOUCH SENSOR:",e);
            op.telemetry.update();
        }
    }

    /*public void addTimer(String timerName){
        try{
            this.timers.put(timerName,new ElapsedTime());
        } catch (Exception e) {
            op.telemetry.addData("ERROR WHILE ADDING TIMER: ",e);
            op.telemetry.update();
        }
    }*/

    public void resetAllEncoders(){
        for (String s:driveMotors.keySet()){
            DcMotor.RunMode m = driveMotors.get(s).getMode();
            driveMotors.get(s).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors.get(s).setMode(m);
        }
    }
    public void setAllRunWithEncoder(){
        for (String s:driveMotors.keySet()){
            driveMotors.get(s).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void setAllRunWithoutEncoder(){
        for (String s:encoderDriveMotors.keySet()){
            driveMotors.get(s).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
    public void setAllRunToPosition(){
        for (String s:driveMotors.keySet()){
            driveMotors.get(s).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setEncoderDriveMotorsRunToPosition(){
        for(String s: encoderDriveMotors.keySet()){
            encoderDriveMotors.get(s).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setAllPower(double power){
        for (String s:driveMotors.keySet()){
            driveMotors.get(s).setPower(power);
        }
    }

    public boolean isAnyMotorBusy(){
        for (String s:driveMotors.keySet()) {
            if (driveMotors.get(s).isBusy()){
                return true;
            }
            if (op.opModeIsActive()){
                return false;
            }
        }
        return false;
    }

    public void waitAllMotorsFree(){
        while(op.opModeIsActive()&&isAnyMotorBusy()){
            //waiting...
        }
    }
    public void stopAllThreads(){
        for (String s : threads.keySet()){
            try{
                threads.get(s).interrupt();
            } catch (Exception e){
                //Do Nothing
            }
        }
    }
}