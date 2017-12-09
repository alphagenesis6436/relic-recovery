package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Kalvin on 12/8/2017.
 *
 * Solely test the drive motors of GeorgeOp
 */

@TeleOp(name = "MecanumTestOp", group = "Default")
@Disabled
public class MecanumTestOp extends GeorgeOp {
    //Declare any motors, servos, and sensors

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)

    public MecanumTestOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        driveFR = hardwareMap.dcMotor.get("dfr");
        driveFR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveFL = hardwareMap.dcMotor.get("dfl");
        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBR = hardwareMap.dcMotor.get("dbr");
        driveBR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveBL = hardwareMap.dcMotor.get("dbl");
        driveBL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initialize servos

        //Initialize sensors

        telemetry();
    }
    @Override public void start() {

    }
    @Override public void loop() {
        //Update all the data based on driver input
        updateData();

     /* Clip Variables to make sure they don't exceed their
      * ranged values and Set them to the Motors/Servos */
        //initialization();

        //Show the Real Values of the Data Using Telemetry
        telemetry();
    }

    @Override void updateData() {
        //Add in update methods for specific robot mechanisms
        if (gamepad1.a)
            runConstantSpeed();
        if (gamepad1.b)
            moveForward(0.20, 12 * 3);
        if (gamepad1.x)
            moveForward(-0.20, -(12 * 3));
        if (gamepad1.y) {
            resetEncoders();
            encoderTargetReached = false;
        }
        forwardRightPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        forwardLeftPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        backwardRightPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        backwardLeftPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;

    }

    @Override void initialization() {
        //Clip and Initialize Drive Train
        forwardRightPower = Range.clip(forwardRightPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveFR.setPower(forwardRightPower);
        forwardLeftPower = Range.clip(forwardLeftPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveFL.setPower(forwardLeftPower);
        backwardRightPower = Range.clip(backwardRightPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveBR.setPower(backwardRightPower);
        backwardLeftPower = Range.clip(backwardLeftPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveBL.setPower(backwardLeftPower);
    }
    @Override void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("FR Pwr", String.format("%.2f",driveFR.getPower()));
        telemetry.addData("FR Encoder", driveFR.getCurrentPosition());
        telemetry.addData("FL Pwr", String.format("%.2f",driveFL.getPower()));
        //telemetry.addData("FL Encoder", driveFL.getCurrentPosition());
        telemetry.addData("BR Pwr", String.format("%.2f",driveBR.getPower()));
        //telemetry.addData("BR Encoder", driveBR.getCurrentPosition());
        telemetry.addData("BL Pwr", String.format("%.2f",driveBL.getPower()));
        //telemetry.addData("BL Encoder", driveBL.getCurrentPosition());
        telemetry.addData("EncoderTargetReached", encoderTargetReached);
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


    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

