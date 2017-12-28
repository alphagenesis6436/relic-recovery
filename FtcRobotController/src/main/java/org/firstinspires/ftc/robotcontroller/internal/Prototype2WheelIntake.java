package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex on 6/1/2017.
 *
 * servos should rotate at same angle from the vertical
 * wheels should be at least 5.9 inches away from each other
 * servos should not rotate too quickly
 */

@TeleOp(name = "Wheel Intake Prototype", group = "Default")
public class Prototype2WheelIntake extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor leftWheel;
    DcMotor rightWheel;
    Servo leftServo; //rename
    Servo rightServo;

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final double WHEEL_PWR_MAX = 0.40;
    final double WHEEL_DELTA = 5 / 255.0f;
    final float LEFT_MIN = 0 / 255.0f;
    final float LEFT_MID = 0.39f;
    final float RIGHT_MID = 0.61f;
    final float RIGHT_MAX = 250 / 255.0f;

    double wheelPower = 0;
    double leftServoPos = LEFT_MIN;
    double rightServoPos = RIGHT_MAX;

    public Prototype2WheelIntake() {}

    @Override public void init() {
        //Initialize motors & set direction
        leftWheel = hardwareMap.dcMotor.get("lw");
        leftWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightWheel = hardwareMap.dcMotor.get("rw");
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize servos
        leftServo = hardwareMap.servo.get("ls");
        rightServo = hardwareMap.servo.get("rs");

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
        if (gamepad2.left_stick_y < 0) {        //out
            leftServoPos += WHEEL_DELTA;
            rightServoPos -= WHEEL_DELTA;
        }
        else if (gamepad2.left_stick_y > 0) {   //in
            leftServoPos -= WHEEL_DELTA;
            rightServoPos += WHEEL_DELTA;
        }
        wheelPower = -gamepad2.right_stick_y * WHEEL_PWR_MAX;
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        wheelPower = Range.clip(wheelPower, -WHEEL_PWR_MAX, WHEEL_PWR_MAX);
        leftWheel.setPower(wheelPower);
        rightWheel.setPower(wheelPower);

        leftServoPos = Range.clip(leftServoPos, LEFT_MIN, LEFT_MID);
        rightServoPos = Range.clip(rightServoPos, RIGHT_MID, RIGHT_MAX);
        leftServo.setPosition(leftServoPos);
        rightServo.setPosition(rightServoPos);
    }

    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Wheel Power", String.format("%.2f", wheelPower));
        telemetry.addData("Left Servo Pos", String.format("%.2f", leftServoPos));
        telemetry.addData("Right Servo Pos", String.format("%.2f", rightServoPos));
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