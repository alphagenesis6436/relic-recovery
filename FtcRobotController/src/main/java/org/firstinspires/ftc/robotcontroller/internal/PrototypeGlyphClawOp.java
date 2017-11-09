package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex on 11/5/2017.
 * Jessica's prototype
 * servos: leftClaw, rightClaw
 * motor: lift
 */

@TeleOp(name = "Glyph Prototype 1", group = "Default")
public class PrototypeGlyphClawOp extends OpMode {
    //Declare any motors, servos, and sensors
    Servo leftClaw; //180
    Servo rightClaw; //180
    DcMotor lift; //Andymark 60:1

    //variables & constants pertaining to claw servos
    final float SERVO_MIN_LEFT = 92 / 255.0f; //left claw is closed
    final float SERVO_MAX_LEFT = 255 / 255.0f; //left claw is open
    final float SERVO_MIN_RIGHT = 0 / 255.0f; //right claw is open
    final float SERVO_MAX_RIGHT = 255 / 255.0f; //right claw is closed
    double leftClawServoPos = SERVO_MIN_LEFT;
    double rightClawServoPos = SERVO_MAX_RIGHT;
    double clawDelta = 0.001;

    //variables & constants pertaining to lift motor
    final float LIFT_PWR_MAX = 0.70f;
    double liftPower = 0;

    public PrototypeGlyphClawOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        lift = hardwareMap.dcMotor.get("lift");

        //Initialize servos
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");

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
        updateGlyphClaw();
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms - initialize servo positions
        leftClawServoPos = Range.clip(leftClawServoPos, SERVO_MIN_LEFT, SERVO_MAX_LEFT);
        leftClaw.setPosition(leftClawServoPos);
        rightClawServoPos = Range.clip(rightClawServoPos, SERVO_MIN_RIGHT, SERVO_MAX_RIGHT);
        rightClaw.setPosition(rightClawServoPos);
        liftPower = Range.clip(liftPower, -LIFT_PWR_MAX, LIFT_PWR_MAX);
        lift.setPower(liftPower);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Current leftClaw servo pos:", String.format("%.0f", leftClawServoPos * 255));
        telemetry.addData("Current rightClaw servo pos:", String.format("%.0f", rightClawServoPos * 255));
        telemetry.addData("Current lift motor power", String.format("%.2f", liftPower));
    }

    //Controlled by Driver 2
    //Step 1: Open Left/Right Claw by pressing the Left/Right Bumper
    //Step 2: Close the Left/Right Claw by pressing the Left/Right Trigger
    void updateGlyphClaw() {
        liftPower = -gamepad2.left_stick_y * LIFT_PWR_MAX;
        if (gamepad2.left_bumper)
            leftClawServoPos += clawDelta;
        else if (gamepad2.left_trigger > 0.50)
            leftClawServoPos -= clawDelta;
        if (gamepad2.right_trigger > 0.50)
            rightClawServoPos += clawDelta;
        else if (gamepad2.right_bumper)
            rightClawServoPos -= clawDelta;
    }


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

