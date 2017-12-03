package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Alex on 11/8/2017.
 * Autonomous Objectives:
 * --Knock Off Jewel for 30 Points
 * --Score 1 Glyph into 1st Column of CryptoBox for 15 points
 * Pseudocode:
 * 0. Start on balancing stone, jewel mechanism faces the jewel
 * 1. Knock off jewel
 * 2. Move left 32 inches towards the blue tape in order to be centered at the cryptobox
 * 3. Rotate 90 degrees counterclockwise to have glyph face CryptoBox
 * 4. Drive right 12 inches to be centered in front of cryptobox
 * 5. Drive forward toward CryptoBox until 4 inches away from the wall
 * 6. Drive right until white tape is seen
 * 7. Drive forward 3?? inches -> push glyph into the CryptoBox
 * End. Robot ends up aligned to score glyph in specific column of CryptoBox
 */
@Autonomous(name = "GeorgeBlue2Auto", group = "default")
//@Disabled
public class GeorgeBlue2Auto extends GeorgeOp {
    //Declare and Initialize any variables needed for this specific autonomous program

    public GeorgeBlue2Auto() {}

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateName, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));
        telemetry();

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateName = "Initial Calibration";
                calibrateVariables();
                resetEncoders();
                state++;
                break;

            case 2:
                stateName = "Knock off jewel 1 - arm down";
                updateVuforia();
                leftClawServoPos = SERVO_GRAB_LEFT;
                leftClaw.setPosition(leftClawServoPos);
                rightClawServoPos = SERVO_GRAB_RIGHT;
                rightClaw.setPosition(rightClawServoPos);
                //upDownServo moves down to max/min?? position
                upDownPos -= 0.03;
                upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
                upDownServo.setPosition(upDownPos);
                colorSensor.enableLed(true);
                if (upDownServo.getPosition() == UPDOWN_MIN || colorSensor.red() >= RED_THRESHOLD
                        || colorSensor.blue() >= BLUE_THRESHOLD)
                    state++;
                break;

            case 4:
                stateName = "Knock off jewel 2 - arm knock";
                updateVuforia();
                if (!waitSec(2)) {//bring up glyph
                    glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    glyphLift.setPower(0.30);
                }
                else
                    glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //if leftJewel == red, leftRightServo moves right to knock off red jewel
                //if leftJewel == blue, leftRightServo moves left to knock off red jewel
                colorSensor.enableLed(true);//Turns Color Sensor into Active Mode
                if (colorSensor.red() >= RED_THRESHOLD)
                    leftRightPos = LEFTRIGHT_MIN;
                else if (colorSensor.blue() >= BLUE_THRESHOLD)
                    leftRightPos = LEFTRIGHT_MAX;
                else if (waitSec(1)) //Fail Safe: If looking into hole
                    leftRightPos -= 0.005;
                leftRightPos = Range.clip(leftRightPos, LEFTRIGHT_MIN, LEFTRIGHT_MAX);
                leftRightServo.setPosition(leftRightPos);
                if (leftRightServo.getPosition() == LEFTRIGHT_MAX || leftRightServo.getPosition() == LEFTRIGHT_MIN) {
                    state++;
                    glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;

            case 6:
                stateName = "Knock off jewel 3 - arm up";
                updateVuforia();
                //leftRight servo moves back to center of robot
                if (leftRightPos != LEFTRIGHT_MID)
                    leftRightPos = LEFTRIGHT_MID;
                leftRightServo.setPosition(leftRightPos);

                //upDownServo moves up to max/min?? position
                upDownPos += 0.03;
                upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
                upDownServo.setPosition(upDownPos);
                if (upDownServo.getPosition() == UPDOWN_MAX)
                    state++;
                break;

            case 8:
                stateName = "Drive Left 32 inches";
                //have robot drive to position of 36 inches
                moveRight(-0.80);
                telemetry.addData("FR Encoder", driveFR.getCurrentPosition() / COUNTS_PER_INCH_RF);
                if (waitSec(1) && !driveFR.isBusy())
                    state++;
                break;

            case 10:
                stateName = "Rotate 90 Degrees Counterclockwise";
                //have robot turn counterclockwise 90 degrees
                turnClockwise(-90);
                if (turnAbsolute(-90))
                    state++;
                break;

            case 12:
                stateName = "Drive Right 12 inches";
                //have robot drive right to position 12 inches
                moveRight(0.2);
                if (waitSec(1) && !driveFR.isBusy())
                    state++;
                break;

            case 14:
                stateName = "Drive forward until 7 inches away from Wall";
                //have robot drive forward until 7 inches away from wall
                moveForward(0.25);
                if (range.getDistance(DistanceUnit.INCH) <= 7)
                    state = 1000;
                break; //End here for league meet 0

            case 16:
                stateName = "Drive right until white tape is seen";
                //have robot drive right until white tape is seen
                moveRight(0.40);
                if (range.getLightDetected() >= WHITE_THRESHOLD)
                    state = 1000;
                break;

            case 18:
                stateName = "Drive forward 3 inches";
                //have robot drive to position of 3 inches forward to score glyph
                moveForward(3);
                if (waitSec(1) && !driveFR.isBusy())
                    state = 1000;
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
                if (waitSec(1)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}