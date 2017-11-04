package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex on 6/1/2017.
 */
@TeleOp(name = "MecanumDriveOp", group = "Default")
public class MecanumDriveOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor driveFR;
    DcMotor driveFL;
    DcMotor driveBR;
    DcMotor driveBL;

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final double DRIVE_PWR_MAX = 0.90;
    double forwardRightPower = 0;
    double forwardLeftPower = 0;
    double backwardRightPower = 0;
    double backwardLeftPower = 0;

    public MecanumDriveOp() {}

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
        updateDriveTrain();
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        forwardRightPower = Range.clip(forwardRightPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveFR.setPower(forwardRightPower);
        forwardLeftPower = Range.clip(forwardLeftPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveFL.setPower(forwardLeftPower);
        backwardRightPower = Range.clip(backwardRightPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveBR.setPower(backwardRightPower);
        backwardLeftPower = Range.clip(backwardLeftPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveBL.setPower(backwardLeftPower);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("FR", forwardRightPower);
        telemetry.addData("FL", forwardLeftPower);
        telemetry.addData("BR", backwardRightPower);
        telemetry.addData("BL", forwardLeftPower);
    }

    //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */

    void updateDriveTrain() {
        forwardRightPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        forwardLeftPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        backwardRightPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        backwardLeftPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * DRIVE_PWR_MAX;
    }


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

    void resetEncoders() {
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void runConstantSpeed() {
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void runConstantPower() {
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void stopDriveMotors() {
        driveFR.setPower(0.0);
        driveFL.setPower(0.0);
        driveBR.setPower(0.0);
        driveBL.setPower(0.0);
    }
    void moveForward(double power) {
        driveFR.setPower(power);
        driveFL.setPower(power);
        driveBR.setPower(power);
        driveBL.setPower(power);
    }
    void moveRight(double power) {
        driveFR.setPower(-power);
        driveFL.setPower(power);
        driveBR.setPower(power);
        driveBL.setPower(-power);
    }
    void moveForwardRight(double power) {
        driveFR.setPower(0.0);
        driveFL.setPower(power);
        driveBR.setPower(power);
        driveBL.setPower(0.0);
    }
    void moveForwardLeft(double power) {
        driveFR.setPower(power);
        driveFL.setPower(0.0);
        driveBR.setPower(0.0);;
        driveBL.setPower(power);
    }
    void turnClockwise(double power) {
        driveFR.setPower(-power);
        driveFL.setPower(power);
        driveBR.setPower(-power);
        driveBL.setPower(power);
    }

    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

