package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Kalvin on 11/4/2017.
 * Jessica's prototype
 * servos: leftClaw, rightClaw
 * motor: lift
 */

@TeleOp(name = "Glyph Prototype 1", group = "Default")
public class PrototypeGlyphClawOp extends OpMode {
    //Declare any motors, servos, and sensors
    Servo leftClaw;
    Servo rightClaw;
    DcMotor lift;

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final float SERVO_MIN_LEFT = 0 / 255.0f;
    final float SERVO_MAX_LEFT = 255 / 255.0f;
    final float SERVO_MIN_RIGHT = 0 / 255.0f;
    final float SERVO_MAX_RIGHT = 255 / 255.0f;
    final float LIFT_PWR_MAX = 0.50f;

    //make sure both servos / motors are in sync
    public double leftClawServoPos = 0;
    public double rightClawServoPos = 0;
    public double liftPower = 0;

    public PrototypeGlyphClawOp() {}

    @Override public void init() {
        //Initialize motors & set direction

        //Initialize servos
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        lift = hardwareMap.dcMotor.get("lift");

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
            leftClawServoPos = (((leftClawServoPos * 255.0) + 1) / 255.0);
        }
        if (gamepad1.left_stick_x > 0) {
            leftClawServoPos = (((leftClawServoPos * 255.0) - 1) / 255.0);
        }
        if (gamepad1.right_stick_x < 0) {
            rightClawServoPos = (((rightClawServoPos * 255.0) + 1) / 255.0);
        }
        if (gamepad1.right_stick_x > 0) {
            rightClawServoPos = (((rightClawServoPos * 255.0) - 1) / 255.0);
        }
        if (gamepad2.left_stick_y == 0) {
            liftPower = 0;
        }
        liftPower = -gamepad2.left_stick_y * LIFT_PWR_MAX;
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms - initialize servo positions
        leftClawServoPos = Range.clip(leftClawServoPos, SERVO_MIN_LEFT, SERVO_MAX_LEFT);
        leftClaw.setPosition(leftClawServoPos);
        rightClaw.setPosition(clawServoPos);
        lift.setPower(liftPower);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Current leftClaw servo pos:", String.format("%.0f", clawServoPos * 255));
        telemetry.addData("Current rightClaw servo pos:", String.format("%.0f", clawServoPos * 255));
        telemetry.addData("Current lift motor power", String.format("%.0f", liftPower * 255));
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

