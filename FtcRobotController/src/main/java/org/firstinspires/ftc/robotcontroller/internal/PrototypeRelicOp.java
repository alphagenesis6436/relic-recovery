package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Kalvin on 11/4/2017.
 * Vivian's prototype
 *
 * 1. set initial position of upDown servo to be upright
 * 1b. set initial position of openClose to be @ top of arc
 * 2. upDowm rotates down (makes sideways arc)
 * 3. openClose rotates down about 90 degrees
 * 4. openClose rotates back up
 */

@TeleOp(name = "Relic Prototype 1", group = "Default")
@Disabled
public class PrototypeRelicOp extends OpMode {
    //Declare any motors, servos, and sensors
    Servo upDownServo;    //180
    Servo openCloseServo; //180

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final float SERVO_MIN = 0 / 255.0f;
    final float SERVO_MAX = 255 / 255.0f;
    double upDownServoPos = 0;
    double openCloseServoPos = 0;

    public PrototypeRelicOp() {}

    @Override public void init() {
        //Initialize motors & set direction

        //Initialize servos
        upDownServo = hardwareMap.servo.get("upDown");
        openCloseServo = hardwareMap.servo.get("openClose");

        //Initialize sensors

        telemetry();
    }
    @Override public void loop() {
        //Update all the data based on driver input
        updateData();

     /* Clip Variables to make sure they don't exceed their
      * ranged values and Set them to the Motors/Servos */
        initialization();

        //Show the Real Values of the Data Using Telemetry
        telemetry();
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        if (gamepad1.left_stick_x < 0) {
            upDownServoPos = (((upDownServoPos * 255.0) + 1) / 255.0);
        }
        if (gamepad1.left_stick_x > 0) {
            upDownServoPos = (((upDownServoPos * 255.0) - 1) / 255.0);
        }
        if (gamepad1.right_stick_y < 0) {
            openCloseServoPos = (((openCloseServoPos * 255.0) + 1) / 255.0);
        }
        if (gamepad1.right_stick_y > 0) {
            openCloseServoPos = (((openCloseServoPos * 255.0) - 1) / 255.0);
        }
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        upDownServoPos = Range.clip(upDownServoPos, SERVO_MIN, SERVO_MAX);
        upDownServo.setPosition(upDownServoPos);        //replace w/ actual initial position

        openCloseServoPos = Range.clip(openCloseServoPos, SERVO_MIN, SERVO_MAX);
        openCloseServo.setPosition(openCloseServoPos);  //replace w/ actual initial position
    }

    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Current upDown servo pos:", String.format("%.0f", upDownServoPos * 255));
        telemetry.addData("Current openClose servo pos:", String.format("%.0f", openCloseServoPos * 255));
    }

    //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */

    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

    void resetEncoders() {

    }
    void runEncoders() {

    }
    void runWithoutEncoders() {

    }
    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

