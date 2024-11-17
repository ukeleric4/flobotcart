package org.firstinspires.ftc.teamcode.huskylens;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriverControl {
    DcMotor L;
    DcMotor R;
    // Speeds for motors
    public double LSpeed = -0.7;
    public double LRotateSpeed = -1;
    public double RSpeed = 0.7;
    public double RRotateSpeed = 1;

    // Instantiates L and R motors from the hardware map
    public DriverControl(HardwareMap hw) {
        L = hw.get(DcMotor.class, "left");
        R = hw.get(DcMotor.class, "right");

        L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Gives user easy control of the robot when in driver mode
     * Left stick has priority over right stick
     * @param left_stick_y_power value between -1 (down) and 1 (up)
     * @param right_stick_x_power value between -1 (left) and 1 (right)
     */
    public void drive(double left_stick_y_power, double right_stick_x_power) {
        if (left_stick_y_power < 0) {
            moveBackward();
        } else if (left_stick_y_power > 0) {
            moveForward();
        } else if (right_stick_x_power > 0) {
            rotateClockwise();
        } else if (right_stick_x_power < 0) {
            rotateCounterClockwise();
        } else {
            stopMovement();
        }
    }

    // Movement functions below
    public void moveForward() {
        R.setPower(RSpeed);
        L.setPower(LSpeed);
    }

    public void moveBackward() {
        R.setPower(-RSpeed);
        L.setPower(-LSpeed);
    }

    public void stopMovement() {
        R.setPower(0);
        L.setPower(0);
    }

    public void rotateClockwise() {
        R.setPower(RRotateSpeed);
        L.setPower(-LRotateSpeed);
    }

    public void rotateCounterClockwise() {
        R.setPower(-RRotateSpeed);
        L.setPower(LRotateSpeed);
    }
}
