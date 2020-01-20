package org.firstinspires.ftc.teamcode.Libraries;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;

/**
 * Created by kk200 on 10/1/2017.
 */

public class RobotRecorder {
    public volatile ArrayList<String> actions;
    public volatile Joystick joy;
    public volatile LinearOpMode op;
    public volatile boolean isRecording = false;
    public volatile boolean isPlaying = false;

    private String fileName;
    private DataHandler dataHandler;
    private captureJoystick runner;
    private playbackRecording playback;
    private Thread recorder;
    private Thread player;

    private final boolean DEBUG = true;
    private EnhancedTelemetry telem;

    public RobotRecorder(final String fileName, final Joystick joy, final LinearOpMode op){
        actions = new ArrayList<>();
        this.fileName = fileName;
        this.joy = joy;
        dataHandler = new DataHandler(this.fileName,op);
        this.op = op;
        telem = new EnhancedTelemetry(op,3,"log.txt");
    }

    public RobotRecorder(final String fileName,final LinearOpMode op){
        this.fileName = fileName;
        this.op = op;
        dataHandler = new DataHandler(this.fileName,op);
        telem = new EnhancedTelemetry(op,3,"log.txt");
    }

    public void beginRecording(){
        //Add check if joy is null
        actions.add(GamepadToString(joy.gamepad,SystemClock.uptimeMillis()));

        isRecording = true;
        runner = new captureJoystick();
        recorder = new Thread(runner,"Capture");
        recorder.start();
    }

    public void stopRecording(){
        isRecording = false;
        recorder.interrupt();
        saveRecording();
    }

    public void cancelRecording(){
        isRecording = false;
        recorder.interrupt();
    }

    public void stopPlayback(){
        isPlaying = false;
        player.interrupt();
    }

    private void saveRecording(){
        telem.addData("Debug","Writing...",2);
        for (int i = 0; i < actions.size(); i++) {
            //JSONObject jsonObject = toJSON(actions.get(i));
            telem.addData("Debug2","Action on: " + i + "/" + actions.size(),1);
            telem.addData("Data:",actions.get(i),2);
            dataHandler.write(actions.get(i));

        }
        telem.addData("Debug","Complete!",2);
    }

    public void playbackRecording(){
        isPlaying = true;
        playback = new playbackRecording();
        player = new Thread(playback,"Playback");
        player.start();
    }

    public void decodeRecording() throws InterruptedException {
        ArrayList<String> data = dataHandler.getData();
        for (int i = 0; i < data.size(); i++){
            if (isPlaying) {
                if (i != 0) {
                    long wait = timeOfString(data.get(i)) - timeOfString(data.get(i-1));
                    telem.addData("Waiting:", wait,2);
                    Thread.sleep(wait);
                }
                telem.addData("Decoded: ", StringToGamepad(data.get(i)),3);
                joy.set(StringToGamepad(data.get(i)));
            }
        }
    }

    public Gamepad StringToGamepad(String input){
        Gamepad obj = new Gamepad();
        String[] parts = input.split(",");

        obj.left_stick_x = Float.parseFloat(parts[0]);
        obj.left_stick_y = Float.parseFloat(parts[1]);
        obj.right_stick_x = Float.parseFloat(parts[2]);
        obj.right_stick_y = Float.parseFloat(parts[3]);
        obj.left_trigger = Float.parseFloat(parts[4]);
        obj.right_trigger = Float.parseFloat(parts[5]);
        obj.left_stick_button = Boolean.parseBoolean(parts[6]);
        obj.right_stick_button = Boolean.parseBoolean(parts[7]);
        obj.a = Boolean.parseBoolean(parts[8]);
        obj.x = Boolean.parseBoolean(parts[9]);
        obj.y = Boolean.parseBoolean(parts[10]);
        obj.b = Boolean.parseBoolean(parts[11]);
        obj.right_bumper = Boolean.parseBoolean(parts[12]);
        obj.left_bumper = Boolean.parseBoolean(parts[13]);
        obj.dpad_up = Boolean.parseBoolean(parts[14]);
        obj.dpad_down = Boolean.parseBoolean(parts[15]);
        obj.dpad_right = Boolean.parseBoolean(parts[16]);
        obj.dpad_left = Boolean.parseBoolean(parts[17]);

        return obj;
    }

    public long timeOfString(String input){
        String[] parts = input.split(",");
        return Long.parseLong(parts[18]);
    }

    public String GamepadToString(Gamepad gamepad, long time){
        String total;
        total = "" + gamepad.left_stick_x;
        total = total + "," + gamepad.left_stick_y;
        total = total + "," + gamepad.right_stick_x;
        total = total + "," + gamepad.right_stick_y;
        total = total + "," + gamepad.left_trigger;
        total = total + "," + gamepad.right_trigger;
        total = total + "," + gamepad.left_stick_button;
        total = total + "," + gamepad.right_stick_button;
        total = total + "," + gamepad.a;
        total = total + "," + gamepad.x;
        total = total + "," + gamepad.y;
        total = total + "," + gamepad.b;
        total = total + "," + gamepad.right_bumper;
        total = total + "," + gamepad.left_bumper;
        total = total + "," + gamepad.dpad_up;
        total = total + "," + gamepad.dpad_down;
        total = total + "," + gamepad.dpad_right;
        total = total + "," + gamepad.dpad_left;
        total = total + "," + time;
        return total;
    }

    public class captureJoystick implements Runnable{
        @Override
        public void run()  {
            telem.addData("Recorder","Live",1);
            while (isRecording){
                telem.addData("Input",op.gamepad1.toString(),3);
                if (GamepadToString(joy.gamepad,timeOfString(actions.get(actions.size()-1))).equals(actions.get(actions.size()-1))) {
                    telem.addData("Detector:","Same Input",3);
                } else {
                    telem.addData("Detector: ","Different Input!",3);
                    actions.add(GamepadToString(joy.gamepad,SystemClock.uptimeMillis()));
                }

            }
            telem.addData("Recorder","Offline",1);
        }
    }

    public class playbackRecording implements Runnable{
        @Override
        public void run() {
            try {
                decodeRecording();
            } catch (InterruptedException e) {
                e.printStackTrace();
                telem.addData("ERROR",e,0);
            }
        }
    }
}
