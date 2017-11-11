package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Kalvin Chang on 9/29/2017.
 *
 */
@TeleOp(name = "Jewel Prototype 2", group = "Default")
@Disabled
public class PrototypeJewelOpMode2 extends OpMode {
    //Declare any motors, servos, and sensors
    final float UPDOWN_MIN = 0 / 255.0f;       //update later
    final float UPDOWN_MAX = 255 / 255.0f;
    final float LEFTRIGHT_MIN = 0 / 255.0f;
    final float LEFTRIGHT_MAX = 255 / 255.0f;
    Servo upDownServo;
    Servo leftRightServo;
    ColorSensor color;

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    public double upDownServoPos = 0;
    public double leftRightServoPos = 0;

    public PrototypeJewelOpMode2() {}

    @Override public void init() {
        //Initialize servos
        upDownServo = hardwareMap.servo.get("upDown");
        leftRightServo = hardwareMap.servo.get("leftRight");
        color = hardwareMap.colorSensor.get("color");
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
        updateActuators();
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        upDownServoPos = Range.clip(upDownServoPos, UPDOWN_MIN, UPDOWN_MAX);
        upDownServo.setPosition(upDownServoPos);
        leftRightServoPos = Range.clip(leftRightServoPos, LEFTRIGHT_MIN, LEFTRIGHT_MAX);
        leftRightServo.setPosition(leftRightServoPos);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Current upDown servo pos:", String.format("%.0f", upDownServoPos * 255));
        telemetry.addData("Current leftRight servo pos:", String.format("%.0f", leftRightServoPos * 255));
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

    /*
    * gamepad1.left_stick_x < 0 = leftRightServo turns left
    * gamepad1.left_stick_x > 0 = leftRightServo turns right
    * gamepad1.right_stick_y > 0 = upDownServoPos moves down
    * gamepad1.right_stick_y < 0 = upDownServoPos moves up
    */
    void updateActuators() {
        if (gamepad1.left_stick_x < 0) {
            leftRightServoPos = (((leftRightServoPos * 255.0) + 1) / 255.0);
        }
        if (gamepad1.left_stick_x > 0) {
            leftRightServoPos = (((leftRightServoPos * 255.0) - 1) / 255.0);
        }
        if (gamepad1.right_stick_y < 0) {
            upDownServoPos = (((upDownServoPos * 255.0) + 1) / 255.0);
        }
        if (gamepad1.right_stick_y > 0) {
            upDownServoPos = (((upDownServoPos * 255.0) - 1) / 255.0);
        }
    }
}
