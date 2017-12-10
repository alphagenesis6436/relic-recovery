package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
 * 1. Knock off jewel (and read Pictograph, and lift Glyph 2 inches up)
 * 2. Drive backward to drive off balancing stone
 * 3. Rotate to 90 degrees to be perpendicular with the walls
 * 4. Drive forward slowly to position needed to score in correct column
 * 5. Rotate to 165 degrees to have glyph face CryptoBox
 * 6. Drive forward toward CryptoBox until glyph is scored
 * 7. Release the glyph
 * 8. Rotate to 135 degrees to push glyph
 * 9. Rotate to 195 degrees to push glyph
 * 10. Rotate to 180 degrees to be perpendicular with wall
 * 11. Drive backward a little bit to park
 */
@Autonomous(name = "GeorgeRed2Auto", group = "default")
//@Disabled
public class GeorgeRed2Auto extends GeorgeOp {
    //Declare and Initialize any variables needed for this specific autonomous program

    public GeorgeRed2Auto() {}

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

            case 2: //Use PID Control to ensure servo moves down slowly and safely
                stateName = "Knock off jewel 1 - arm down";
                updateVuforia();
                leftClawServoPos = SERVO_GRAB_LEFT;
                leftClaw.setPosition(leftClawServoPos);
                rightClawServoPos = SERVO_GRAB_RIGHT;
                rightClaw.setPosition(rightClawServoPos);
                leftClawTopServoPos = SERVO_GRAB_LEFT_TOP;
                topLeftClaw.setPosition(leftClawTopServoPos);
                rightClawTopServoPos = SERVO_GRAB_RIGHT_TOP;
                topRightClaw.setPosition(rightClawTopServoPos);
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

                //if leftJewel == red, leftRightServo moves left to knock off red jewel
                //if leftJewel == blue, leftRightServo moves right to knock off red jewel
                colorSensor.enableLed(true);//Turns Color Sensor into Active Mode
                if (colorSensor.red() >= RED_THRESHOLD)
                    leftRightPos = LEFTRIGHT_MAX;
                else if (colorSensor.blue() >= BLUE_THRESHOLD)
                    leftRightPos = LEFTRIGHT_MIN;
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
                if (!waitSec(2.5)) {//bring up glyph
                    glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    glyphLift.setPower(0.30);
                }
                else
                    glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //leftRight servo moves back to center of robot
                if (leftRightPos != LEFTRIGHT_MID)
                    leftRightPos = LEFTRIGHT_MID;
                leftRightServo.setPosition(leftRightPos);

                //upDownServo moves up to max/min?? position
                upDownPos += 0.03;
                upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
                upDownServo.setPosition(upDownPos);
                if (upDownServo.getPosition() == UPDOWN_MAX && waitSec(2.5))
                    state++;
                break;

            case 8:
                stateName = "Drive backward to drive off balancing stone";
                moveForward(-0.20, -1.5);
                if (encoderTargetReached)
                    state++;
                break;

            case 10:
                stateName = "Rotate to 90 degrees to be perpendicular with the walls";
                //have robot drive forward until 6 inches away from wall
                turnClockwise(90);
                if (turnAbsolute(90))
                    state++;
                break;

            case 12:
                stateName = "Drive forward until correct column reached";
                if (pictographKey == 0) { //drive to left column
                    moveForward(0.20, 2.75);
                }
                else if (pictographKey == 1) { //drive to middle column
                    moveForward(0.20, 1.75);
                }
                else if (pictographKey == 2) { //drive to right column
                    moveForward(0.20, 0.75);
                }
                if (encoderTargetReached) {
                    state++;
                    pictographKey = -1;
                }
                break;

            case 14:
                stateName = "Rotate to -165 degrees to have glyph face CryptoBox";
                turnClockwise(165);
                if (turnAbsolute(165))
                    state++;
                break;

            case 16:
                stateName = "Drive forward toward CryptoBox until glyph is scored";
                moveForward(0.20);
                if (range.getDistance(DistanceUnit.INCH) <= 6)
                    state++;
                break;

            case 18:
                stateName = "Drop and Release Glyph";
                //have robot drive to position of 3 inches forward to score glyph
                leftClawServoPos = SERVO_MIN_LEFT;
                leftClaw.setPosition(leftClawServoPos);
                rightClawServoPos = SERVO_MAX_RIGHT;
                rightClaw.setPosition(rightClawServoPos);
                leftClawTopServoPos = SERVO_MIN_LEFT_TOP;
                topLeftClaw.setPosition(leftClawTopServoPos);
                rightClawTopServoPos = SERVO_MAX_RIGHT_TOP;

                topRightClaw.setPosition(rightClawTopServoPos);
                if (leftClaw.getPosition() == SERVO_MIN_LEFT && rightClaw.getPosition() == SERVO_MAX_RIGHT)
                    state = 26;
                break;

            case 20:
                stateName = "Rotate to 135 degrees to push glyph";
                turnClockwise(135);
                if (turnAbsolute(135))
                    state++;
                break;

            case 22:
                stateName = "Rotate to 195 degrees to push glyph";
                turnClockwise(195);
                if (turnAbsolute(195))
                    state++;
                break;

            case 24:
                stateName = "Rotate to 180 degrees to be perpendicular with wall";
                turnClockwise(180);
                if (turnAbsolute(180))
                    state++;
                break;

            case 26:
                stateName = "Drive backward a little bit to park";
                moveForward(-0.20);
                if (waitSec(1))
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