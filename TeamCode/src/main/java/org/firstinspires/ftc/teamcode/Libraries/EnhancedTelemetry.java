package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Kenneth on 10/2/2017.
 */

public class EnhancedTelemetry {
    private String logFileName;
    private int DEBUGLevel = 0;
    private OpMode op;
    DataHandler logHandler;

    public EnhancedTelemetry(OpMode o, int DebugLev, String filename){
        op = o;
        DEBUGLevel = DebugLev;
        this.logFileName = filename;
        String dateTime = (new java.util.Date(System.currentTimeMillis())).toString();
        logHandler = new DataHandler(logFileName);
        logToFile("------Starting Telemetry at [" + dateTime + "]-----");
    }

    public String getLogFileName() {
        return logFileName;
    }

    public void setLogFileName(String logFileName) {
        this.logFileName = logFileName;
    }

    public int getDEBUGLevel() {
        return DEBUGLevel;
    }

    public void setDEBUGLevel(int DEBUGLevel) {
        this.DEBUGLevel = DEBUGLevel;
    }

    public OpMode getOp() {
        return op;
    }

    public void setOp(OpMode op) {
        this.op = op;
    }

    public void error(Exception e){
        addData("ERROR: ", e.toString(),0);
        logToFile("ERROR: ",e);
    }
    public void lowpriority(String key, Object msg){
        addData(key,msg.toString(),3);
    }

    public void warn(Object o){
        addData("WARN: ",o.toString(),1);
        logToFile("WARN: ",o.toString());
    }
    public void notify(Object o){
        addData("NOTIFY: ", o.toString(),2);
    }

    public void addData(String key, Object value,int level){
        if(DEBUGLevel >= level){
            op.telemetry.addData(key, value.toString());
            op.telemetry.update();
        }
    }
    public void logToFile(String msg) {
        try {
            logHandler.write(msg);

        }catch (Exception e){
            op.telemetry.addData("Error", e);
        }
    }
    public void logToFile(String key, Object value) {
        try {
            logHandler.write(key + value.toString());

        }catch (Exception e){
            op.telemetry.addData("Error", e);
        }
    }

}