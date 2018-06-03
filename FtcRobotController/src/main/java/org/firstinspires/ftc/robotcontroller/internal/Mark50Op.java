package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Alex on 4/27/2017.
 */

@TeleOp(name = "Mark50Op", group = "Default")
@Disabled
public class Mark50Op extends OpMode {
    //Declare any motors on robot
    DcMotor motor1;
    DcMotor motor2;

    public Mark50Op() {}

    @Override public void init() {
        //Initialize motors & set direction
        motor1 = hardwareMap.dcMotor.get("lm");
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2 = hardwareMap.dcMotor.get("rm");
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    @Override public void loop() {
        //Update all the data based on driver input
        updateData();
    }

    void updateData() {
        //Add in update methods for specific robot mechanisms
        updateTankDrive2(motor1, motor2, 0.75);
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

    }
}

