package org.firstinspires.ftc.teamcode.Libraries;


import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Scanner;

/**
 * Created by kk200 on 9/30/2017.
 */

public class DataHandler {
    private PrintWriter writer;
    private File path;
    private File fileToGet;
    private OpMode opMode;

    public DataHandler(String fileName) {
        path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS);
        fileToGet = new File(path, cleanName(fileName));
        try {
            writer = new PrintWriter(new FileWriter(fileToGet,true));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public DataHandler(String fileName, OpMode op) {
        path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS);
        fileToGet = new File(path, cleanName(fileName));
        try {
            writer = new PrintWriter(new FileWriter(fileToGet,true));
        } catch (Exception e) {
            e.printStackTrace();
            op.telemetry.addData("ERROR",e);
            op.telemetry.update();
        }
        this.opMode = op;
    }

    private String cleanName(String fileName){
        if (fileName.substring(fileName.length() - 4,fileName.length()).equals(".txt")) {
            return fileName;
        }
        return fileName + ".txt";
    }

    public void write(String data){ writer.println(data); }

    public ArrayList<String> getData(){
        Scanner data = null;
        ArrayList<String> compiledData = new ArrayList<>();
        try {
            data = new Scanner(fileToGet);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        while (data.hasNextLine()){
            compiledData.add(data.nextLine());
        }

        return compiledData;
    }
}

