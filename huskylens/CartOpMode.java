package org.firstinspires.ftc.teamcode.huskylens;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class CartOpMode extends OpMode {
     HuskyLensCart huskyLensCart;
     DriverControl driverControl;
     CartDoor cartDoor;
     private boolean aPressed = false;
     private boolean huskyLensModeActivated = false;
     private boolean driverControlActivated = true;

     @Override
     public void init() {
          huskyLensCart = new HuskyLensCart(hardwareMap);
          driverControl = new DriverControl(hardwareMap);
          cartDoor = new CartDoor(hardwareMap);
     }

     @Override
     public void loop() {
          double runtime = getRuntime();

          // Put in place to enable user to only have to press a once to switch modes
          if (gamepad1.a && !aPressed) {
               if (!huskyLensModeActivated)  {
                    driverControlActivated = false;
                    huskyLensModeActivated = true;
               } else {
                    driverControlActivated = true;
                    huskyLensModeActivated = false;
               }
          }

          if (huskyLensModeActivated) {
              huskyLensCart.getObservedObject(1, runtime, !gamepad1.b, telemetry);
          }

          if (driverControlActivated) {
              driverControl.drive(gamepad1.left_stick_y, gamepad1.right_stick_x);
              // If user is holding x, door will close (couldn't find a solution with runtime)
              if (gamepad1.x) {
                   cartDoor.doorAction("close");
              } else if (gamepad1.y) { // If user is holding y, door will open
                   cartDoor.doorAction("open");
              } else { // If user is not holding x or y, door will remain in most recent position
                   cartDoor.doorAction("stop");
              }
          }
          aPressed = gamepad1.a;
     }
}
