package org.firstinspires.ftc.teamcode.huskylens;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CartDoor {
    public DcMotor hexMotor;

    public CartDoor(HardwareMap hw) {
        hexMotor = hw.get(DcMotor.class, "hexmotor");
    }

    public void doorAction(String direction) {
        switch (direction) {
            // Used to determine door state based off passed in 'direction'
            case "open":
                hexMotor.setPower(1.0);
                break;
            case "close":
                hexMotor.setPower(-1.0);
                break;
            case "stop":
                hexMotor.setPower(0);
                break;
        }
    }
}
