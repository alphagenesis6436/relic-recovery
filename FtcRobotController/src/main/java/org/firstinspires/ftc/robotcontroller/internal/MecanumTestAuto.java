package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Alex on 11/8/2017.
 * Autonomous Objectives:
 * --Knock Off Jewel for 30 Points
 * --Score 1 Glyph (15 points) in Correct Column CryptoBox for (30 points)
 * --Park in Safe Zone (10 points)
 * Pseudocode:
 * 0. Start on balancing stone, jewel mechanism faces the jewel
 * 1. Knock off jewel (and read Pictograph, and lift Glyph 2 inches up)
 * 2. Drive forward to drive off balancing stone
 * 3. Drive backward to allign with balancing stone
 * 4. Drive forward slowly to position needed to score in correct column
 * 5. Rotate to -75 degrees to have glyph face CryptoBox
 * 6. Drive forward toward CryptoBox until glyph is scored
 * 7. Release the glyph
 * 8. Rotate to -105 degrees to push glyph
 * 9. Rotate to -45 degrees to push glyph
 * 10. Rotate to -90 degrees to be perpendicular with wall
 * 11. Drive backward a little bit to park
 * End. Robot ends up aligned to score glyph in specific column of CryptoBox
 */
@Autonomous(name = "MecanumTestAuto", group = "default")
public class MecanumTestAuto extends GeorgeOp {
    //Declare and Initialize any variables needed for this specific autonomous program

    public MecanumTestAuto() {}

    @Override
    public void loop() {
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateName, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));
        telemetry.addData("FR Pwr", String.format("%.2f",driveFR.getPower()));
        telemetry.addData("FR Encoder", driveFR.getCurrentPosition());
        telemetry.addData("FL Pwr", String.format("%.2f",driveFL.getPower()));
        telemetry.addData("FL Encoder", driveFL.getCurrentPosition());
        telemetry.addData("BR Pwr", String.format("%.2f",driveBR.getPower()));
        telemetry.addData("BR Encoder", driveBR.getCurrentPosition());
        telemetry.addData("BL Pwr", String.format("%.2f",driveBL.getPower()));
        telemetry.addData("BL Encoder", driveBL.getCurrentPosition());
        telemetry.addData("EncoderTargetReached", encoderTargetReached);

        switch (state) {
            case 0:
            stateName = "Initial Calibration";
            calibrateVariables();
            resetEncoders();
            state++;
            break;

            case 2:
                stateName = "Drive FR-Wheel fwd 2 revolution";
                driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFR.setPower(0.20);
                if (1120 * 2 - driveFR.getCurrentPosition() <= 10)
                    state++;
                break;

            case 4:
                stateName = "Drive FL-Wheel fwd 2 revolution";
                driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFL.setPower(0.20);
                if (1120 * 2 - driveFL.getCurrentPosition() <= 10)
                    state++;
                break;

            case 6:
                stateName = "Drive BR-Wheel fwd 2 revolution";
                driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBR.setPower(0.20);
                if (1120 * 2 - driveBR.getCurrentPosition() <= 10)
                    state++;
                break;

            case 8:
                stateName = "Drive BL-Wheel fwd 2 revolution";
                driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBL.setPower(0.20);
                if (1120 * 2 - driveBL.getCurrentPosition() <= 10)
                    state++;
                break;

            case 10:
                stateName = "Drive FR & FL fwd 2 revolutions";
                driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFR.setPower(0.20);
                driveFL.setPower(0.20);
                if (1120 * 2 - driveFL.getCurrentPosition() <= 10)
                    state++;
                break;

            case 12: stateName = "Drive FR & BR fwd 2 revolutions";
                driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFR.setPower(0.20);
                driveBR.setPower(0.20);
                if (1120 * 2 - driveBR.getCurrentPosition() <= 10)
                    state++;
                break;

            case 14: stateName = "Drive FR & BL fwd 2 revolutions";
                driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                driveFR.setPower(0.20);
                driveBL.setPower(0.20);
                if (1120 * 2 - driveBL.getCurrentPosition() <= 10)
                    state++;
                break;

            case 1000: //Run When Autonomous is Complete
                stateName = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                calibrateVariables();
                resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateName = "Calibrating";
                calibrateVariables();
                resetEncoders();
                if (waitSec(2)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

}