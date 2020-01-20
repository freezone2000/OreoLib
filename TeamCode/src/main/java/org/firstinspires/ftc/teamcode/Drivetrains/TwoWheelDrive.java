package org.firstinspires.ftc.teamcode.Drivetrains;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Libraries.IMUHelper;
import org.firstinspires.ftc.teamcode.Libraries.Joystick;
import org.firstinspires.ftc.teamcode.Libraries.PID;

/**
 * Created by kk200 on 10/23/2017.
 */

public class TwoWheelDrive extends Drivetrain {


    public String backLeftName = "BLM";
    public String backRightName = "BRM";
    public String imuName = "CustomIMU";

    public Thread driveWJ;
    public Thread backwardsDriveWJ;

    // -----------------VARIABLES--------------------
    public final static double COUNTS_PER_MOTOR_REV = 500;    // eg: TETRIX Motor Encoder
    public final static double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public final static double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public final static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public final double ticksPerInch = 25.907;
    // -----------------VARIABLES--------------------
    public PID pid;



    public TwoWheelDrive(LinearOpMode op) {
        super(op);


        addEncoderDriveMotor(backLeftName);
        addEncoderDriveMotor(backRightName);
        addGyro(imuName);
        //addCustomGyro(imuName);
        motors.get(backLeftName).setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setDrivePower(double power){


        motors.get(backLeftName).setPower(power);
        motors.get(backRightName).setPower(power);
    }

    public void setTurnPower(double power){


        motors.get(backLeftName).setPower(power);
        motors.get(backRightName).setPower(-power);
    }

    public void startDriveWithJoystick(final Joystick joy){
        driveWJ = new Thread(new Runnable() {
            @Override
            public void run() {
                float speed = 1;
                while(op.opModeIsActive()){
                    speed = 1;

                    motors.get(backRightName).setPower(speed * -(joy.rightY() + joy.rightX()));
                    motors.get(backLeftName).setPower(speed * -(joy.rightY() - joy.rightX()));


                }
            }
        });
        addThread(driveWJ);
        driveWJ.start();
    }
    public void startBackwardsDriveWithJoystick(final Joystick joy){
        backwardsDriveWJ = new Thread(new Runnable() {
            @Override
            public void run() {
                float speed = 1;
                while(op.opModeIsActive()){
                    motors.get(backRightName).setPower(speed * (joy.rightY() + joy.leftX()));
                    motors.get(backLeftName).setPower(speed * (joy.rightY() - joy.leftX()));
                }
            }
        },"Backwards Drive");
        addThread(backwardsDriveWJ);
        backwardsDriveWJ.start();
    }

    public void stopDriveWithJoystick(){
        driveWJ.interrupt();
    }
    /*
        public void strafeWithGyro(double distance, double speed, double trimAngle){
            IMUHelper imu = gyros.get(imuName);
            pid = new PID(op,0,0.054, 0.0,0.048);
            if(!imu.IMUupdateIsRunning()){ //Make sure IMU is running, if not, start IMU
                imu.updater.interrupt();
                imu.startIMU();
                op.telemetry.addData("Error!", "IMU NOT STARTED BEFORE STRAIGHT!");
            }
            trimAngle += imu.getYaw();//NEW
            setAllRunToPosition(); //Set motors to use Run to position mode
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            int newLeftFrontTarget = motors.get(frontLeftName).getCurrentPosition() + moveCounts;
            int newRightFrontTarget = motors.get(frontRightName).getCurrentPosition() - moveCounts;
            int newLeftBackTarget = motors.get(backLeftName).getCurrentPosition() - moveCounts;
            int newRightBackTarget = motors.get(backRightName).getCurrentPosition() + moveCounts;
            motors.get(frontLeftName).setTargetPosition(newLeftFrontTarget);
            motors.get(backLeftName).setTargetPosition(newLeftBackTarget);
            motors.get(frontRightName).setTargetPosition(newRightFrontTarget);
            motors.get(backRightName).setTargetPosition(newRightBackTarget);
            //Set speed for all motors
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            double steer;
            double rightSpeed,leftSpeed;
            double max;
            while (op.opModeIsActive() && isAnyMotorBusy()){
                steer = pid.update(imu.getYaw(),trimAngle);
                if (distance < 0)
                    steer *= -1.0;
                leftSpeed = speed + steer;
                rightSpeed = speed - steer;
                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }
                setEachWheel(leftSpeed,-rightSpeed,-leftSpeed,rightSpeed);
                op.telemetry.addData("Steer",  "%5.1f",   steer);
                op.telemetry.addData("Target Ticks",  "%7d:%7d",      newLeftFrontTarget,  newRightFrontTarget);
                op.telemetry.addData("Current Ticks",  "%7d:%7d",      motors.get(frontLeftName).getCurrentPosition(),motors.get(frontRightName).getCurrentPosition());
                op.telemetry.addData("Speed (L,R)",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                op.telemetry.addData("Raw Heading",imu.getYaw());
                op. telemetry.update();
            }
            setSpeed(0,0);
            setAllRunWithEncoder();
        }
    */
    public void straightWithGyro(double distance, double speed, double trimAngle) {
        IMUHelper imu = gyros.get(imuName);
        pid = new PID(op,0,0, 0,0);


        if(!imu.IMUupdateIsRunning()){ //Make sure IMU is running, if not, start IMU
            imu.updater.interrupt();
            imu.startIMU();
            op.telemetry.addData("Error!", "IMU NOT STARTED BEFORE STRAIGHT!");
        }


        trimAngle += imu.getYaw();//NEW

        //resetAllEncoders(); //Reset Encoder ticks to 0
        //setAllRunToPosition(); //Set motors to use Run to position mode
        //int moveCounts = (int)(distance * COUNTS_PER_INCH);
        int moveCounts = INtoTicks(distance);

        int newLeftBackTarget = motors.get(backLeftName).getCurrentPosition() + moveCounts;

        int newRightBackTarget = motors.get(backRightName).getCurrentPosition() + moveCounts;



        motors.get(backLeftName).setTargetPosition(newLeftBackTarget);

        motors.get(backRightName).setTargetPosition(newRightBackTarget);
        setAllRunToPosition();

        //Set speed for all motors
        speed = Range.clip(Math.abs(speed), 0.0, 1);
        setSpeed(speed,speed);
        double steer;
        double rightSpeed,leftSpeed;
        double max;
        while (op.opModeIsActive() && isAnyMotorBusy()){
            steer = pid.update(imu.getYaw(),trimAngle);

            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed + steer;
            rightSpeed = speed - steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0) {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            setSpeed(leftSpeed,rightSpeed);

            op.telemetry.addData("Steer",  "%5.1f",   steer);
            op.telemetry.addData("Target Ticks",  "%7d:%7d");
            op.telemetry.addData("Current Ticks",  "%7d:%7d");
            op.telemetry.addData("Speed (L,R)",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            op.telemetry.addData("Raw Heading",imu.getYaw());
            op. telemetry.update();
        }
        setSpeed(0,0);

        setAllRunWithEncoder();
    }



    /*public void gyroPointTurn(double angleToTurn, double speed, double tolerance){
        IMUHelper imu = gyros.get(imuName);
        pid = new PID(op,0,0.08,0.0,0.048);
        if(!imu.IMUupdateIsRunning()){ //Make sure IMU is running, if not, start IMU
            imu.updater.interrupt();
            imu.startIMU();
        }
        double steer = pid.update(imu.getYaw(),angleToTurn);
        double leftSpeed,rightSpeed;
        leftSpeed = speed + steer;
        rightSpeed = speed - steer;
        while (op.opModeIsActive() && steer != 0) {
            setSpeed(leftSpeed,rightSpeed);
            op.telemetry.addData("Current Heading:",imu.getYaw());
            op.telemetry.addData("Desired:",angleToTurn);
            op.telemetry.addData("Steer:",steer);
            op.telemetry.addData("Speed: (L,R):",leftSpeed + ","+rightSpeed);
            op.telemetry.update();
        }
        setSpeed(0,0);
    }*/

    public void gyroPointTurn(double angleToTurn, double speed, double angleTolerance, boolean isAbsolute){
        op.telemetry.addData("GyroTurnStatus","Starting");
        double startYaw, targetYaw,leftPower,rightPower;
        speed = Range.clip(speed,-1,1);
        IMUHelper imu = gyros.get(imuName);
        if(!imu.IMUupdateIsRunning()){
            op.telemetry.addData("ERROR START UR IMU BEFOR U TURN","Error code 3.1415");
            imu.updater.interrupt();
            imu.startIMU();
        }

        if(angleToTurn > 0 && angleToTurn < 360){
            leftPower = speed;
            rightPower = -speed;
        }else if(angleToTurn < 0 && angleToTurn > -360){
            leftPower = -speed;
            rightPower = speed;
        }else{
            op.telemetry.addData("ERROR: CHEK UR ANGLE B4 U REK ","URSELF");
            return;
        }
        startYaw = imu.getYaw();
        double minAngleToTurn = Math.abs(angleToTurn) - angleTolerance;

        if(!isAbsolute) {
            targetYaw = angleToTurn + startYaw;
        }else{
            targetYaw = angleToTurn;
        }

        if (targetYaw > 360) {
            targetYaw %= 360;
        } else if (targetYaw < 0) {
            targetYaw += 360;
        }

        op.telemetry.addData("Current Yaw",imu.getYaw());
        op.telemetry.addData("Start Yaw",startYaw);
        op.telemetry.addData("Min Angle To Turn",minAngleToTurn);
        op.telemetry.addData("GyroTurnStatus","Turning");
        op.telemetry.update();
        while(op.opModeIsActive() && (IMUHelper.distanceBetweenAngles(imu.getYaw(),startYaw) < minAngleToTurn)){
            setSpeed(leftPower,rightPower);
            op.telemetry.addData("Current Yaw",imu.getYaw());
            op.telemetry.addData("Start Yaw",startYaw);
            op.telemetry.addData("Min Angle To Turn",minAngleToTurn);
            op.telemetry.addData("GyroTurnStatus","Turning");
            op.telemetry.update();
        }

        op.telemetry.addData("GyroTurnStatus","Complete!");
        op.telemetry.update();

        leftPower = 0;
        rightPower = 0;

        setSpeed(leftPower,rightPower);
    }



    public void setSpeed(double leftSpeed,double rightSpeed){

        motors.get(backRightName).setPower(rightSpeed);

        motors.get(backLeftName).setPower(leftSpeed);
    }

    public void setEachWheel(double frontLeft, double backLeft, double frontRight, double backRight){

        motors.get(backRightName).setPower(backRight);

        motors.get(backLeftName).setPower(backLeft);
    }

    public void encoderDrive(double leftdist,double rightdist, double power){
        encoderDriveTicks(INtoTicks(leftdist),INtoTicks(rightdist),power);
    }

    public void encoderPointTurn(double distIN, double power){
        encoderDrive(distIN,-distIN,power);
    }

    public void encoderDriveTicks(int leftTicks,int rightTicks, double power){


        DcMotor BLM = motors.get(backLeftName);
        DcMotor BRM = motors.get(backRightName);

        //resetAllEncoders();
        //setAllRunToPosition();


        //BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        BLM.setTargetPosition(BLM.getCurrentPosition() + leftTicks);
        BRM.setTargetPosition(BRM.getCurrentPosition() + rightTicks);

        BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (leftTicks < 0 || rightTicks < 0) {
            power = power * -1;
        }


        BLM.setPower(power);
        BRM.setPower(power);

        //op.telemetry.addData("Pwrs ", FLM.getPower() + ", "+ FRM.getPower());
        //op.telemetry.update();

        op.telemetry.addData("EncoderTicks","Busy start");
        op.telemetry.update();

        op.telemetry.addData("EncoderTicks","Done!");
        op.telemetry.update();
        power = 0;


        BLM.setPower(power);
        BRM.setPower(power);
    }

    public void encoderDriveStraight(double distIN, double power){


        DcMotor BLM = motors.get(backLeftName);
        DcMotor BRM = motors.get(backRightName);


        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderDrive(distIN,distIN,power);
    }

    public int INtoTicks(double inches){
        return (int)(inches * ticksPerInch);
    }

    public double getError(double targetAngle,double curAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - curAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public void StraightWithTimer(double runTime, double power){
        DcMotor BLM = motors.get(backLeftName);
        DcMotor BRM = motors.get(backRightName);

        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double initialTime =System.currentTimeMillis();
        while (System.currentTimeMillis() - initialTime < runTime) {

            BLM.setPower(power);
            BRM.setPower(power);
        }

        BLM.setPower(0);
        BRM.setPower(0);

    }
}