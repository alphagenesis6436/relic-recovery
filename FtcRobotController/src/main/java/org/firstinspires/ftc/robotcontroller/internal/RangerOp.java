package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex on 6/22/2017.
 */
@TeleOp(name = "RangerOp", group = "Default")
@Disabled
public class RangerOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor leftDrive;
    DcMotor rightDrive;
    //Declare any variables & constants pertaining to Drive Train
    final double DRIVE_PWR_MAX = 0.80;
    double currentLeftPwr = 0.0;
    double currentRightPwr = 0.0;
    double drive = 0.0;
    double turn = 0.0;

    public RangerOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        leftDrive = hardwareMap.dcMotor.get("ld");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive = hardwareMap.dcMotor.get("rd");
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
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

    void initialization() { //Clip and Initilaize All Output Systems
        //Clip and Initialize Drive Train
        currentLeftPwr = Range.clip(currentLeftPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        leftDrive.setPower(currentLeftPwr);

        currentRightPwr = Range.clip(currentRightPwr, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        rightDrive.setPower(currentRightPwr);

}
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Left Drive Pwr", leftDrive.getPower());
        telemetry.addData("Drive Drive Pwr", rightDrive.getPower());
    }

    //Create Methods that will update the driver data

 /*
     All update methods should be commented with:
         //Controlled by Driver (1 or 2)
         //Step 1: Push Up/Down on the Left/Right Control Stick to Drive Forward/Backward
         //Step 2: (Physical Instructions on how to control specific robot mechanism using controller buttons)
         //Step ...: (Physical Instructions on how to control specific robot mechanism using controller buttons)
  */

    //Controlled by Driver 1
    //Step 1: Push Up/Down on the Left/Right Control Stick to Drive Forward/Backward
    void updateDriveTrain() {
        drive = -gamepad1.left_stick_y * DRIVE_PWR_MAX;
        turn  =  gamepad1.left_stick_x * DRIVE_PWR_MAX;
        currentLeftPwr = drive + turn;
        currentRightPwr = drive - turn;

    }

    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

    void resetEncoders() {
        //leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void runConstantSpeed() {
        //leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void runWithoutPower() {
        //leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void resetSensors() {

    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

    void driveForward(double power) {
        //power = Range.clip(power, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        //leftDrive.setPower(power);
        //rightDrive.setPower(power);
    }

}
