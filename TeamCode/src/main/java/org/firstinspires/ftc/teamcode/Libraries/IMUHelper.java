package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by kk200 on 11/3/2017.
 */

public class IMUHelper {
    public BNO055IMU imu;
    public volatile Orientation angles;
    public volatile Acceleration gravity;
    public LinearOpMode op;


    public IMUHelper(LinearOpMode op, BNO055IMU i){
        this.op = op;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = i;
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
    }
    public Thread updater = new Thread(new Runnable() {
        @Override public void run() {
            while(op.opModeIsActive()) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        }
    });

    public String getCalibrationStatus(){
        return imu.getCalibrationStatus().toString();
    }

    public void startAccelerationLogging() { // Use after init
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    public String getHeading(){

        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    public String getRoll(){
        return formatAngle(angles.angleUnit, angles.secondAngle);
    }

    public String getPitch(){
        return formatAngle(angles.angleUnit, angles.thirdAngle);
    }

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void startIMU(){
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        updater.start();
    }
    public boolean IMUupdateIsRunning(){
        return updater.isAlive();
    }

    public double[] readIMU(){
        return new double[]{Double.parseDouble(getHeading()),Double.parseDouble(getPitch()),Double.parseDouble(getRoll())};
    }
    public double getYaw(){

        double newYaw = 0.0;
        double curYaw = readIMU()[0];
        //op.telemetry.addData("curYaw",curYaw);
        //op.telemetry.update();
        // Note:  The gyro outputs values from 0 to 180 degrees and -180 to 0 degrees as the
        // robot spins clockwise. Convert this to a 0 to 360 degrees scale.
        if (curYaw > -180 && curYaw < 0) {
            newYaw = 360 + curYaw;
        } else {
            newYaw = curYaw;
        }
        return (newYaw);
    }

    public static double distanceBetweenAngles(double angle1, double angle2) {
        // Both angle1 and angle2 are assumed to be positive numbers between 0 and 360
        // The returnValue is between 0 and 180.
        double angleDistance= Math.abs(angle1 - angle2);

        if (angleDistance > 180) {
            angleDistance = 360 - angleDistance;
        }

        return (angleDistance);
    }


}