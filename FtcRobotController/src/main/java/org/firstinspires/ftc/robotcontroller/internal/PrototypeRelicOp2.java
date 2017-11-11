package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Kalvin on 11/10/2017.
 * winchMotor - Drawer slides
 * relic claw - relic grabber (code pulled from PrototypeRelicOp)
 */

@TeleOp(name = "Relic Prototype 2", group = "Default")
@Disabled
public class PrototypeRelicOp2 extends PrototypeRelicOp {
    //Declare any motors, servos, and sensors
    DcMotor winchMotor;

    //Declare any variables & constants pertaining to specific robot mechanisms (i.e. drive train)
    double WINCH_PWR_MAX = 0.80;
    double winchPower = 0;

    public PrototypeRelicOp2() {}

    @Override public void init() {
        super.init();
        //Initialize motors & set direction
        winchMotor = hardwareMap.dcMotor.get("winch");

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

    @Override void updateData() {
        super.updateData();
        winchPower = gamepad2.left_stick_y * WINCH_PWR_MAX;
        winchMotor.setPower(winchPower);
    }

    @Override void initialization() {
        super.initialization();
        winchPower = Range.clip(winchPower, -WINCH_PWR_MAX, WINCH_PWR_MAX);
        winchMotor.setPower(winchPower);
    }

    @Override void telemetry() {
        //Show Data for Specific Robot Mechanisms
        super.telemetry();
        telemetry.addData("Current winchMotor power:", String.format("%.2f", winchPower));
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

