package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Karim Karim on 1/10/2018.
 */

public class PID {
    private LinearOpMode op;

    private double errorThreshold = 0;
    private double error = 0;
    private double errorSum = 0;
    private double prevError = 0;
    private double prevTime = 0;
    private double dt = 0;

    //---------------PID VARIABLES---------------
    public volatile double kp = 0;
    public volatile double ki = 0;
    public volatile double kd = 0;
    //---------------PID VARIABLES---------------

    public PID(LinearOpMode op,double errorThreshold, double KP, double KI, double KD) {
        this.op = op;
        this.errorThreshold = errorThreshold;
        prevTime = System.currentTimeMillis();
        kp = KP;
        kd = KD;
        ki = KI;
    }

    public void updateConstants(double KP, double KI, double KD){
        this.kp = KP;
        this.ki = KI;
        this.kd = KD;
    }

    double output;
    public double update(double current, double target) {
        if (prevTime == 0) {
            prevTime = System.currentTimeMillis();
        }
        error = getError(current,target);
        dt = System.currentTimeMillis() - prevTime;
        errorSum += (error * dt);
        output = (error * kp) + (errorSum * ki) + (((error - prevError)/dt) * kd);
        prevError = error;
        prevTime = System.currentTimeMillis();
        return output;
    }

    public static double getError(double current, double target) {
        double difference = current - target;

        if (difference > 180) {
            difference -= 360;
        }

        return difference;
    }
}