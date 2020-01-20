package org.firstinspires.ftc.teamcode.Drivetrains;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Libraries.IMUHelper;
import org.firstinspires.ftc.teamcode.Libraries.Joystick;
import org.firstinspires.ftc.teamcode.Libraries.PID;


public class MecanumDrivetrain extends FourWheelDrive{
    public String frontRightName = "FRM";
    public String backLeftname = "BLM";
    public String backRightName = "BRM";

    public Thread driveWJThread;


    public static double valueP = 0;

    public MecanumDrivetrain(LinearOpMode op){
        super(op);
        //addThread(driveWJThread);
    }


    public void startDriveWithJoystick(final Joystick joy1, final Joystick joy2){
        driveWJThread = new Thread(new Runnable() {
            @Override
            public void run() {
                setAllRunWithEncoder();



                DcMotor frm = motors.get(frontRightName);
                DcMotor flm = motors.get(frontLeftName);
                DcMotor brm = motors.get(backRightName);
                DcMotor blm = motors.get(backLeftname);
                double frmPower,flmPower,blmPower,brmPower = 0;
                boolean slowEnabled = false;
                boolean prevStateBumper = false;
                double slowFactor = 0.569;
                double speedMultiplier = .8;

                double jiggle = 0;
                double jiggleDirection = 1;
                long startCycleTime = System.currentTimeMillis();
                boolean jiggleStillPressed = false;
                int cyclePeriodMS = 50;
                op.telemetry.addData("Drive","Run called");
                op.telemetry.update();
                while (op.opModeIsActive()){
                    op.telemetry.addData("OUTPUT3","Running...");
                    op.telemetry.update();

                    /*
                    if(joy1.buttonX()||joy2.buttonB()) {
                        if(!jiggleStillPressed) {
                            jiggle = 0.5; //SET JIGGLE HERE!!!
                            jiggleStillPressed = true;
                            startCycleTime = System.currentTimeMillis();
                        }
                        if(System.currentTimeMillis() > startCycleTime + cyclePeriodMS){
                            jiggleDirection *= -1;
                            startCycleTime = System.currentTimeMillis();
                        }
                        jiggle = jiggleDirection * jiggle;
                    }else{
                        jiggleStillPressed = false;
                        jiggle = 0;
                    }
                    */
                    jiggle = 0;

                    frmPower = (-joy1.leftY() - joy1.leftTrigger() + joy1.rightTrigger() + jiggle);
                    flmPower = (-joy1.rightY() + joy1.leftTrigger() - joy1.rightTrigger() - jiggle);
                    brmPower = (-joy1.leftY() + joy1.leftTrigger() - joy1.rightTrigger() + jiggle);
                    blmPower = (-joy1.rightY() - joy1.leftTrigger() + joy1.rightTrigger() - jiggle);

                    if(joy1.rightBumper() != prevStateBumper && joy1.rightBumper()){
                        slowEnabled = !slowEnabled;
                    }
                    prevStateBumper = joy1.rightBumper();
                    if(slowEnabled){
                        frmPower*=slowFactor;
                        flmPower*=slowFactor;
                        blmPower*=slowFactor;
                        brmPower*=slowFactor;
                    } else {
                        frmPower*=speedMultiplier;
                        flmPower*=speedMultiplier;
                        blmPower*=speedMultiplier;
                        brmPower*=speedMultiplier;
                    }


                    frm.setPower(frmPower);
                    flm.setPower(flmPower);
                    brm.setPower(brmPower);
                    blm.setPower(blmPower);
                }
            }
        },"driveWJThread");
        addThread(driveWJThread);
        driveWJThread.start();
    }

    public void stopDriveJoystick() {
        driveWJThread.interrupt();
    }

    public double returnYaw(){
        IMUHelper imu = gyros.get(imuName);
        return imu.getYaw();
    }

    public void encoderDriveTicks(int frontTicks,int backTicks, double power){
        DcMotor FLM = motors.get(frontLeftName);
        DcMotor FRM = motors.get(frontRightName);
        DcMotor BLM = motors.get(backLeftName);
        DcMotor BRM = motors.get(backRightName);

        //resetAllEncoders();
        setAllRunToPosition();
        FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FLM.setTargetPosition(FLM.getCurrentPosition() - frontTicks);
        FRM.setTargetPosition(FRM.getCurrentPosition() - frontTicks);
        BLM.setTargetPosition(BLM.getCurrentPosition() + backTicks);
        BRM.setTargetPosition(BRM.getCurrentPosition() + backTicks);

        double frontPower = power;
        double backPower = power;
        if(frontTicks > 0){
            frontPower *= -1;
        }
        if(backTicks < 0){
            backTicks *= -1;
        }
        FLM.setPower(frontPower);
        FRM.setPower(frontPower);
        BLM.setPower(backPower);
        BRM.setPower(backPower);

        //op.telemetry.addData("Pwrs ", FLM.getPower() + ", "+ FRM.getPower());
        //op.telemetry.update();

        op.telemetry.addData("EncoderTicks","Busy start");
        op.telemetry.update();
        while(FRM.isBusy() || FLM.isBusy()){
            double newPowerL = power/2.0;
            double newPowerR = power/2.0;
            BLM.setPower(newPowerL);
            BRM.setPower(newPowerR);
            op.telemetry.addData("Pwrs ", newPowerL + ", "+ newPowerR);
            op.telemetry.addData("EncoderTicks","Busy...");
            op.telemetry.update();
        }
        op.telemetry.addData("EncoderTicks","Done!");
        op.telemetry.update();
        power = 0;
        FLM.setPower(power);
        FRM.setPower(power);
        BLM.setPower(power);
        BRM.setPower(power);
    }

    public void strafeWithGyro(double distance, double speed){
        //NOTE: POSITIVE DISTANCE IS STRAFE LEFT!!!!!!
        IMUHelper imu = gyros.get(imuName);
        pid = new PID(op,0,.01,0,0);
        if(!imu.IMUupdateIsRunning()){ //Make sure IMU is running, if not, start IMU
            imu.updater.interrupt();
            imu.startIMU();
        }
        double target = imu.getYaw();
        resetAllEncoders(); //Reset Encoder ticks to 0
        setAllRunToPosition(); //Set motors to use Run to position mode
        //int moveCounts = (int)(distance * COUNTS_PER_INCH);
        int moveCounts = INtoTicks(distance);
        int newLeftFrontTarget = motors.get(frontLeftName).getCurrentPosition() + moveCounts;
        int newLeftBackTarget = motors.get(backLeftName).getCurrentPosition() - moveCounts;
        int newRightFrontTarget = motors.get(frontRightName).getCurrentPosition() - moveCounts;
        int newRightBackTarget = motors.get(backRightName).getCurrentPosition() + moveCounts;

        motors.get(frontLeftName).setTargetPosition(newLeftFrontTarget);
        motors.get(backLeftName).setTargetPosition(newLeftBackTarget);
        motors.get(frontRightName).setTargetPosition(newRightFrontTarget);
        motors.get(backRightName).setTargetPosition(newRightBackTarget);

        //Set speed for all motors
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        setSpeed(speed,speed);
        double steer = 0;
        double frontLeftSpeed,frontRightSpeed,backLeftSpeed,backRightSpeed;
        double max;
        double startTime = System.currentTimeMillis();
        while (op.opModeIsActive() && (isAnyMotorBusy())){
            //while (op.opModeIsActive() && System.currentTimeMillis() - startTime < 5000){
            steer = pid.update(imu.getYaw(),target);

            if (distance < 0)
                steer *= -1.0;

            //if (Math.abs(steer) > 0) {

            //}
            frontLeftSpeed = speed - steer;
            frontRightSpeed = speed + steer;
            backLeftSpeed = speed - steer;
            backRightSpeed = speed + steer;

            //Normalize speeds if either one exceeds +/- 1.0;
            /*max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if(max > 1) {
                leftSpeed /= max;
                rightSpeed /= max;
            }*/

            motors.get(frontLeftName).setPower(frontLeftSpeed);
            motors.get(frontRightName).setPower(-frontRightSpeed);
            motors.get(backLeftName).setPower(-backLeftSpeed);
            motors.get(backRightName).setPower(backRightSpeed);

            //setSpeed(leftSpeed,rightSpeed);
            op.telemetry.addData("Steer",   steer);
            op.telemetry.addData("Target Ticks",  "%7d:%7d",      motors.get(frontLeftName).getTargetPosition(),  motors.get(frontRightName).getTargetPosition());
            op.telemetry.addData("Current Ticks",  "%7d:%7d",      motors.get(frontLeftName).getCurrentPosition(),motors.get(frontRightName).getCurrentPosition());
            op.telemetry.addData("Speed (L,R)",   "%5.2f:%5.2f", motors.get(frontLeftName).getPower() , motors.get(frontRightName).getPower());
            op.telemetry.addData("Raw Heading",imu.getYaw());
            op.telemetry.update();
        }
        //gyroPointTurn(target - imu.getYaw(),.2,1,false);
        setSpeed(0,0);
        //op.sleep(500);
        setAllRunWithEncoder();
    }

    public void encoderStrafeTicks(int rightTicks,int leftTicks, double power){
        DcMotor FLM = motors.get(frontLeftName);
        DcMotor FRM = motors.get(frontRightName);
        DcMotor BLM = motors.get(backLeftName);
        DcMotor BRM = motors.get(backRightName);

        //resetAllEncoders();
        setAllRunToPosition();
        FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FLM.setTargetPosition(FLM.getCurrentPosition() - leftTicks);
        FRM.setTargetPosition(FRM.getCurrentPosition() - rightTicks);
        BLM.setTargetPosition(BLM.getCurrentPosition() + leftTicks);
        BRM.setTargetPosition(BRM.getCurrentPosition() + rightTicks);

        double frontPower = power;
        double backPower = power;
        if(rightTicks > 0){
            frontPower *= -1;
        }
        if(leftTicks < 0){
            leftTicks *= -1;
        }
        FLM.setPower(frontPower);
        FRM.setPower(frontPower);
        BLM.setPower(backPower);
        BRM.setPower(backPower);

        //op.telemetry.addData("Pwrs ", FLM.getPower() + ", "+ FRM.getPower());
        //op.telemetry.update();

        op.telemetry.addData("EncoderTicks","Busy start");
        op.telemetry.update();
        while(FRM.isBusy() || FLM.isBusy()){
            double newPowerL = power/2.0;
            double newPowerR = power/2.0;
            BLM.setPower(newPowerL);
            BRM.setPower(newPowerR);
            op.telemetry.addData("Pwrs ", newPowerL + ", "+ newPowerR);
            op.telemetry.addData("EncoderTicks","Busy...");
            op.telemetry.update();
        }
        op.telemetry.addData("EncoderTicks","Done!");
        op.telemetry.update();
        power = 0;
        FLM.setPower(power);
        FRM.setPower(power);
        BLM.setPower(power);
        BRM.setPower(power);
    }
    public void StraightWithTimer(double runTime, double power) {
        DcMotor BLM = motors.get(backLeftName);
        DcMotor BRM = motors.get(backRightName);
        DcMotor FLM = motors.get(frontLeftName);
        DcMotor FRM = motors.get(frontRightName);

        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double initialTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - initialTime < runTime) {

            BLM.setPower(power);
            BRM.setPower(power);
            FRM.setPower(power);
            FLM.setPower(power);

        }
        BLM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        BRM.setPower(0);
    }

}