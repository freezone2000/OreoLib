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

public class FourWheelDrive extends Drivetrain {
    public String frontLeftName = "FLM";
    public String frontRightName = "FRM";
    public String backLeftName = "BLM";
    public String backRightName = "BRM";
    public String imuName = "CustomIMU";

    public Thread driveWJ;

    // -----------------VARIABLES--------------------
    public final static double COUNTS_PER_MOTOR_REV = 537.6;    // eg: Neverest Orbitals
    public final static double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public final static double WHEEL_DIAMETER_INCHES = 3;     // For figuring circumference
    public final static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    // -----------------VARIABLES--------------------
    public PID pid;

    private double initialAngle;

    public FourWheelDrive(LinearOpMode op) {
        super(op);
        addEncoderDriveMotor(frontLeftName);
        addEncoderDriveMotor(frontRightName);
        addEncoderDriveMotor(backLeftName);
        addEncoderDriveMotor(backRightName);
        addGyro(imuName);
        //addCustomGyro(imuName);
        motors.get(backLeftName).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(frontLeftName).setDirection(DcMotorSimple.Direction.REVERSE);

        IMUHelper imu = gyros.get(imuName);
        if(!imu.IMUupdateIsRunning()){
            op.telemetry.addData("ERROR START UR IMU BEFOR U TURN","Error code 3.1415");
            imu.updater.interrupt();
            imu.startIMU();
        }

        initialAngle = imu.getYaw();
    }

    public void setDrivePower(double power){
        motors.get(frontLeftName).setPower(power);
        motors.get(frontRightName).setPower(power);
        motors.get(backLeftName).setPower(power);
        motors.get(backRightName).setPower(power);
    }

    public void setTurnPower(double power){
        motors.get(frontLeftName).setPower(power);
        motors.get(frontRightName).setPower(-power);
        motors.get(backLeftName).setPower(power);
        motors.get(backRightName).setPower(-power);
    }

    public void startDriveWithJoystick(final Joystick joy, final float precisionSpeed){
        driveWJ = new Thread(new Runnable() {
            @Override
            public void run() {
                float speedDif = 1;
                while(op.opModeIsActive()){
                    if(joy.leftBumper())
                        speedDif = precisionSpeed;
                    else speedDif = 1;
                    motors.get(frontRightName).setPower(speedDif * (-joy.rightY() - joy.rightX()));
                    motors.get(frontLeftName).setPower(speedDif * (-joy.rightY() + joy.rightX()));
                    motors.get(backRightName).setPower(speedDif * (-joy.rightY() - joy.rightX()));
                    motors.get(backLeftName).setPower(speedDif * (-joy.rightY() + joy.rightX()));
                }
            }
        });
        addThread(driveWJ);
        driveWJ.start();
    }
    public void stopDriveWithJoystick(){
        driveWJ.interrupt();
    }

    public void straightWithGyro(double distance, double speed){
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
        int newLeftBackTarget = motors.get(backLeftName).getCurrentPosition() + moveCounts;
        int newRightFrontTarget = motors.get(frontRightName).getCurrentPosition() + moveCounts;
        int newRightBackTarget = motors.get(backRightName).getCurrentPosition() + moveCounts;

        motors.get(frontLeftName).setTargetPosition(newLeftFrontTarget);
        motors.get(backLeftName).setTargetPosition(newLeftBackTarget);
        motors.get(frontRightName).setTargetPosition(newRightFrontTarget);
        motors.get(backRightName).setTargetPosition(newRightBackTarget);

        //Set speed for all motors
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        setSpeed(speed,speed);
        double steer = 0;
        double leftSpeed,rightSpeed;
        double max;
        while (op.opModeIsActive() && (isAnyMotorBusy())){
            steer = pid.update(imu.getYaw(),target);

            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            //Normalize speeds if either one exceeds +/- 1.0;
            /*max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if(max > 1) {
                leftSpeed /= max;
                rightSpeed /= max;
            }*/

            setSpeed(leftSpeed,rightSpeed);
            op.telemetry.addData("Steer",   steer);
            op.telemetry.addData("Target Ticks",  "%7d:%7d",      motors.get(frontLeftName).getTargetPosition(),  motors.get(frontRightName).getTargetPosition());
            op.telemetry.addData("Current Ticks",  "%7d:%7d",      motors.get(frontLeftName).getCurrentPosition(),motors.get(frontRightName).getCurrentPosition());
            op.telemetry.addData("Speed (L,R)",   "%5.2f:%5.2f", motors.get(frontLeftName).getPower() , motors.get(frontRightName).getPower());
            op.telemetry.addData("Raw Heading",imu.getYaw());
            op.telemetry.update();
        }
        op.telemetry.addData("Angle to turn: ",target - imu.getYaw());
        op.telemetry.update();
        op.sleep(800);
        setSpeed(0,0);
        setAllRunWithoutEncoder();

        if (Math.abs(IMUHelper.distanceBetweenAngles(imu.getYaw(),target)) > 1.5) {
            gyroPointTurn(IMUHelper.distanceBetweenAngles(imu.getYaw(),target), .1, 1);
        }

    }

    public void gyroPointTurn(double angle,double speed,double tolerance){
        op.telemetry.addData("Gyro Turn","Started");
        op.telemetry.update();

        IMUHelper imu = gyros.get(imuName);
        if(!imu.IMUupdateIsRunning()){
            op.telemetry.addData("ERROR START UR IMU BEFOR U TURN","Error code 3.1415");
            imu.updater.interrupt();
            imu.startIMU();
        }

        double startYaw = imu.getYaw();

        double targetYaw = startYaw + angle;

        if (targetYaw > 360) {
            targetYaw %= 360;
        } else if (targetYaw < 0) {
            targetYaw += 360;
        }


        double distanceToTarget = IMUHelper.distanceBetweenAngles(imu.getYaw(),targetYaw);
        for (int i = 1; i <= 8; i++) {
            if (op.opModeIsActive() && ((distanceToTarget < 0 + tolerance) ^ (distanceToTarget > 0 - tolerance))) {
                while (op.opModeIsActive() && ((distanceToTarget < 0 + tolerance) ^ (imu.getYaw() > 0 - tolerance))) {

                    distanceToTarget = IMUHelper.distanceBetweenAngles(imu.getYaw(),targetYaw);

                    if (distanceToTarget < 0 + tolerance) {
                        if (angle > 0) {
                            setSpeed(-speed / i, speed / i);
                        } else {
                            setSpeed(speed / i, -speed / i);
                        }
                    } else if (distanceToTarget > 0 - tolerance) {
                        if (angle > 0) {
                            setSpeed(speed / i, -speed / i);
                        } else {
                            setSpeed(-speed / i, speed / i);
                        }
                    } else {
                        setSpeed(0, 0);
                        break;
                    }
                    op.telemetry.addData("Distance to target",distanceToTarget);
                    op.telemetry.addData("TARGET","" + targetYaw);
                    op.telemetry.addData("START YAW","" +startYaw);
                    op.telemetry.addData("CURRENT YAW","" + imu.getYaw());
                    op.telemetry.addData("angle:",angle);
                    op.telemetry.addData("iteration:",i);
                    op.telemetry.update();
                }
                op.sleep(200);
            } else {
                break;
            }
        }
        op.telemetry.addData("Gyro Turn","Done!");
        op.telemetry.update();
        setSpeed(0,0);
    }



    public void setSpeed(double leftSpeed,double rightSpeed){
        motors.get(frontRightName).setPower(rightSpeed);
        motors.get(backRightName).setPower(rightSpeed);
        motors.get(frontLeftName).setPower(leftSpeed);
        motors.get(backLeftName).setPower(leftSpeed);
    }

    public void setEachWheel(double frontLeft, double backLeft, double frontRight, double backRight){
        motors.get(frontRightName).setPower(frontRight);
        motors.get(backRightName).setPower(backRight);
        motors.get(frontLeftName).setPower(frontLeft);
        motors.get(backLeftName).setPower(backLeft);
    }

    public void encoderDrive(double leftdist,double rightdist, double power){
        encoderDriveTicks(INtoTicks(leftdist),INtoTicks(rightdist),power);
    }

    public void encoderPointTurn(double distIN, double power){
        encoderDrive(distIN,-distIN,power);
    }

    public void encoderDriveTicks(int leftTicks,int rightTicks, double power){
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

        FLM.setTargetPosition(FLM.getCurrentPosition() + leftTicks);
        FRM.setTargetPosition(FRM.getCurrentPosition() + rightTicks);
        BLM.setTargetPosition(BLM.getCurrentPosition() + leftTicks);
        BRM.setTargetPosition(BRM.getCurrentPosition() + rightTicks);
        if (leftTicks < 0 || rightTicks < 0) {
            power = power * -1;
        }
        FLM.setPower(power);
        FRM.setPower(power);
        BLM.setPower(power);
        BRM.setPower(power);

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

    public void encoderDriveStraight(double distIN, double power){
        DcMotor FLM = motors.get(frontLeftName);
        DcMotor FRM = motors.get(frontRightName);
        DcMotor BLM = motors.get(backLeftName);
        DcMotor BRM = motors.get(backRightName);
        FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderDrive(distIN,distIN,power);
    }


    public int INtoTicks(double inches){
        return (int)(inches * COUNTS_PER_INCH);
    }

    public double getError(double targetAngle,double curAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - curAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
}