package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Kalvin Chang on 9/29/2017.
 *
 * 1. servo arm moves down
 * (next: color sensor)
 * 2. motor rotates mechanism left / right
 */
@TeleOp(name = "Jewel Prototype", group = "Default")
public class PrototypeJewelOpMode extends OpMode {
    //Declare any motors, servos, and sensors
    final float PWR_MAX = 0.30f;
    final float ARM_MIN = 0 / 255.0f;
    final float ARM_MAX = 255 / 255.0f;
    public Servo upDownServo;
    DcMotor leftRightMotor;

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    public double servoPos = 0;
    public double motorPower;

    public PrototypeJewelOpMode() {}

    @Override public void init() {
        //Initialize servos
        upDownServo = hardwareMap.servo.get("servo");
        leftRightMotor = hardwareMap.dcMotor.get("motor");
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
        servoPos = Range.clip(servoPos, ARM_MIN, ARM_MAX);
        upDownServo.setPosition(servoPos);
        motorPower = Range.clip(motorPower, -PWR_MAX, PWR_MAX);
        leftRightMotor.setPower(motorPower);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Current servo pos:", String.format("%.0f", servoPos * 255));
        //leftRightMotor.getCurrentPosition();      turn encoder on?
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
    * gamepad1.left_stick_x < 0 = motor turns left
    * gamepad1.left_stick_x > 0 = motor turns right
    * gamepad1.right_stick_y > 0 = move servoArm down
    * gamepad1.right_stick_y < 0 = move servoArm up
    */
    void updateActuators() {
        if (gamepad1.left_stick_x < 0) {
            motorPower = PWR_MAX * gamepad1.left_stick_x;
        }
        if (gamepad1.left_stick_x > 0) {
            motorPower = PWR_MAX * gamepad1.left_stick_x;
        }
        if (gamepad1.left_stick_x == 0) {
            motorPower = 0;
        }
        if (gamepad1.right_stick_y < 0) {
            //servo increase
            servoPos = (((servoPos * 255.0) + 1) / 255.0);
        }
        if (gamepad1.right_stick_y > 0) {
            //servo decrease
            servoPos = (((servoPos * 255.0) - 1) / 255.0);
        }
    }
}
