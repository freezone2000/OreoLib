package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

/**
 * Created by Karim Karim on 2/6/2018.
 */

public class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
    public BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
        super(simple, isSimpleOwned);
    }

    @Override
    public void setReadWindow(I2cDeviceSynch.ReadWindow window) {
        // intentionally do nothing
    }
}