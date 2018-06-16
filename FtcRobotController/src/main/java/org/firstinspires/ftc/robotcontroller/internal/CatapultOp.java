package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex on 6/2/2018.
 */

@TeleOp(name = "CatapultOp", group = "Default")
//@Disabled
public class CatapultOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor catapultMotor;

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    final double CATAPULT_PWR_MAX = 0.80;
    double catapultPwr = 0.0;

    public CatapultOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        catapultMotor = hardwareMap.dcMotor.get("cm");
        catapultMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
        catapultPwr = -gamepad1.left_stick_y * CATAPULT_PWR_MAX;
    }

    void initialization() {
        //Clip and Initialize Specific Robot Mechanisms
        catapultPwr = Range.clip(catapultPwr, -CATAPULT_PWR_MAX, CATAPULT_PWR_MAX);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("Catapult Pwr", catapultMotor.getPower());
        catapultMotor.setPower(catapultPwr);
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

