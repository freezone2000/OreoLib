package org.firstinspires.ftc.teamcode.Libraries;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by kk200 on 10/5/2017.
 */

public class Vuforia {
    private final String licenseKey = "AdtCIzH/////AAAAGVkbDFcppkWGkqpLjdBEavWJ2uW/CgCrcMd74zFJYJgq1RfL+bjzIAzhefr6rvFBhvoSqYKp8FeNeJgnwNsnJ7qj/XTve5QijLoCzjf/qjXJ0N5wfzLY45ycBm0X7awOau1NcBOrU2/XvQWmawD79QDnHWRlBivh84Qx72CdTHWRA/BoJhXMugJIMolVxQ7kcfJwL6SuYgO5cCB8Vk4SFmNRpb0LwJiNs3ICBULjnLECi3VH4OWAsQGneogMn9I+Ngrq1cKl+ko3Wy2tav0MmD6KPdikSnhQRRyEK6vd93Npntt6p5+XoQ9P7kMx2ERvWVZJIljE6+OsrQgT7S7twchFljtlP4Ou5lRFKbb8gLZO";

    private VuforiaLocalizer vuforia;
    private LinearOpMode op;
    private int cameraMonitorID;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    //private EnhancedTelemetry telemetry;
    private boolean relicTrackableActivated = false;

    public volatile String currentVuMark = null;
    //private String logFile = "VuforiaLog";
    private volatile double readBufferTime = 2; //Seconds

    public Thread runner;

    public Vuforia(LinearOpMode op, boolean showCam){
        this.op = op;
        //this.telemetry = new EnhancedTelemetry(op,3,logFile);

        cameraMonitorID = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters;
        if (showCam) {
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorID); //Adding/Removing cameraMonitorID hides/shows camera view
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }
        parameters.vuforiaLicenseKey = licenseKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // Only helpful for debugging
    }

    private void read(){
        if (!relicTrackableActivated) {
            relicTrackables.activate();
            relicTrackableActivated = true;
        }

        /*RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN){
            op.telemetry.addData("VuMark","VuMark read as: " + vuMark.toString());
            op.telemetry.update();
            currentVuMark = vuMark.toString();
        } else {
            currentVuMark = null;
        }*/
    }

    public void startReader(){
        runner = new Thread(new Runnable() {
            @Override
            public void run() {
                CameraDevice.getInstance().setFlashTorchMode(true);
                //CameraDevice.getInstance().setField("brightnesss","1");
                while(op.opModeIsActive()) {
                    read();
                }
            }
        });
        runner.start();
    }

    private void stopReader(){
        runner.interrupt();
        CameraDevice.getInstance().setFlashTorchMode(false);
    }

    public String getCurrentReading(){
        startReader();
        long curTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - curTime < readBufferTime * 1000)){
            if (op.opModeIsActive() == false){
                break;
            }
            op.telemetry.addData("VuMark","Reading...");
            op.telemetry.update();
        }
        stopReader();
        return currentVuMark;
    }
}