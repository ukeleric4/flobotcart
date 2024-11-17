package org.firstinspires.ftc.teamcode.huskylens;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HuskyLensCart {
    public HuskyLens huskyLens;
    public DistanceSensor distanceSensor;
    public DcMotor L;
    public DcMotor R;
    // Detected objects array
    public HuskyLens.Block[] observedObjects = new HuskyLens.Block[10];
    // Specific object with id 1
    public HuskyLens.Block detectedBody;
    // Speed for robot when in CV mode
    public double LSpeed = 0.7;
    public double LRotateSpeed = 0.7;
    public double RSpeed = -0.7;
    public double RRotateSpeed = -0.7;
    // Variables for CV logic
    public int screenWidth = 320;
    public int maxHeight = 220;
    public int differenceInX = 0;
    public boolean tryDetection;
    public double focalLengthInches = 0.1417;

    // Instantiates CV parts from the hardware map
    public HuskyLensCart(HardwareMap hw) {
        huskyLens = hw.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        distanceSensor = hw.get(DistanceSensor.class, "distancesensor");

        L = hw.get(DcMotor.class, "left");
        R = hw.get(DcMotor.class, "right");
    }

    /**
     * Checks the array of detected objects and runs the detection code on the object with id '1'
     * @param id specified object id to be checked for in detection array
     * @param runTime runtime since 'start' button has been pressed in rev hub
     * @param tryDetectionBoolean decides whether the detection code is run (failsafe)
     */
    public void getObservedObject(int id, double runTime, boolean tryDetectionBoolean, Telemetry telemetry) {
        observedObjects = huskyLens.blocks(id);

        if (observedObjects.length > 0) {
            for (int i = 0; i < observedObjects.length; i++) {
                if (observedObjects[i].id == id) {
                    followDetection(i, telemetry);
                } else if (tryDetection) {
                    stopMovement();
                } else {
                    stopMovement();
                }
            }
        } else {
            stopMovement();
        }
        this.tryDetection = tryDetectionBoolean;
    }

    // Movement functions
    public void moveForward() {
        R.setPower(RSpeed);
        L.setPower(LSpeed);
    }
    public void stopMovement() {
        R.setPower(0);
        L.setPower(0);
    }
    public void rotateClockwise() {
        R.setPower(-RRotateSpeed);
        L.setPower(LRotateSpeed);
    }
    public void rotateCounterClockwise() {
        R.setPower(RRotateSpeed);
        L.setPower(-LRotateSpeed);
    }

    /**
     * Allows for easy access to the distance sensor
     * @param objectSizeInches Size of the object (ideally height) in inches
     * @param objectPixelSize Size of the object (ideally height) in pixels
     * @return Distance in specified unit between distance sensor and nearest object
     */
    public double getDistance(double objectSizeInches, int objectPixelSize) {
        return (((objectSizeInches) * focalLengthInches) / objectPixelSize);
    }

    /**
     * Based on how far the detected object is from the center of the screen, robot will attempt to center it by rotating
     * @param index index of the object with specified id in the 'detected objects' array
     */
    public void followDetection(int index, Telemetry telemetry) {
        detectedBody = observedObjects[index];
        differenceInX = (screenWidth / 2) - detectedBody.x;
        telemetry.addData("Distance", getDistance(38, detectedBody.height));

        // Checks distance of the robot to ensure that the robot doesn't crash into the object
        if (detectedBody.height < maxHeight) {
            // Check if the robot is centered on the detectedBody
            if (differenceInX >= 20) {
                // If the detection is to the left of the screen, rotate clockwise
                rotateCounterClockwise();
            } else if (differenceInX <= -20) {
                // If the detection is to the right of the screen, rotate counterclockwise
                rotateClockwise();
            } else {
                // If the detection isn't to the left or right, move forward
                moveForward();
            }
        } else {
            // If the robot is too close to the object, it will stop moving
            stopMovement();
        }
    }
}
