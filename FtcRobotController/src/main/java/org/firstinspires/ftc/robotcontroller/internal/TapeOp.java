package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Alex on 4/27/2017.
 */

@TeleOp(name = "TapeOp", group = "Default")
public class TapeOp extends OpMode {
    //Declare any motors on robot
    DcMotor rightMotor ;
    DcMotor leftMotor;
    DcMotor armMotor;

    public TapeOp() {}

    @Override public void init() {
        //Initialize motors & set direction
        leftMotor = hardwareMap.dcMotor.get("lm");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor = hardwareMap.dcMotor.get("rm");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor = hardwareMap.dcMotor.get("am");
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    @Override public void loop() {
        //Update all the data based on driver input
        updateData();
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateTankDrive2(leftMotor, rightMotor, 0.75);
        updateArm();
    }

    //Create Methods that will update the driver data
    void updateTankDrive2(DcMotor leftMotor, DcMotor rightMotor, final double DRIVE_PWR_MAX){
        double leftPower = -gamepad1.left_stick_y * DRIVE_PWR_MAX;
        double rightPower = -gamepad1.right_stick_y * DRIVE_PWR_MAX;
        leftPower = Range.clip(leftPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        leftMotor.setPower(leftPower);
        rightPower = Range.clip(rightPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        rightMotor.setPower(rightPower);
    }
    void updateArm() {
        armMotor.setPower(gamepad1.right_trigger);
    }
}

