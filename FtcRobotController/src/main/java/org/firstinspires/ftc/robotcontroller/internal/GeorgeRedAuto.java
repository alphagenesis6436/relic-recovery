package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * 2. Move right 36 inches towards the red tape in order to be centered at the cryptobox
 * 3. Rotate 180 degrees to have glyph face CryptoBox
 * 4. Drive forward toward CryptoBox until 4 inches away from the wall
 * 5. Drive left until white tape is seen
 * 6. Drive forward 3?? inches -> push glyph into the CryptoBox
 * End. Robot ends up aligned to score glyph in specific column of CryptoBox
 */
@Autonomous(name = "GeorgeRedAuto", group = "default")
public class GeorgeRedAuto extends GeorgeOp {
    //Declare and Initialize any variables needed for this specific autonomous program

    public GeorgeRedAuto() {}

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
                //upDownServo moves down to max/min?? position
                upDownPos -= 0.03;
                upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
                upDownServo.setPosition(upDownPos);
                if (upDownServo.getPosition() == UPDOWN_MIN)
                    state++;
                break;

            case 4:
                stateName = "Knock off jewel 2 - arm knock";
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
                if (leftRightServo.getPosition() == LEFTRIGHT_MAX || leftRightServo.getPosition() == LEFTRIGHT_MIN)
                    state++;
                break;

            case 6:
                stateName = "Knock off jewel 3 - arm up";
                //leftRight servo moves back to center of robot
                if (leftRightPos != LEFTRIGHT_MID)
                    leftRightPos = LEFTRIGHT_MID;
                leftRightServo.setPosition(leftRightPos);

                //upDownServo moves up to max/min?? position
                upDownPos += 0.03;
                upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
                upDownServo.setPosition(upDownPos);
                if (upDownServo.getPosition() == UPDOWN_MAX)
                    state = 1000;
                break;

            case 8:
                stateName = "Drive Left 36 inches";
                //have robot drive to position of 36 inches
                moveRight(-36);
                if (waitSec(1) && !driveFR.isBusy())
                    state = 1000;
                break;

            case 10:
                stateName = "Rotate 180 Degrees";
                //have robot turn clockwise 180 degrees
                turnClockwise(0.50);
                if (turnAbsolute(180))
                    state = 1000;
                break;

            case 12:
                stateName = "Drive forward until 4 inches away from Wall";
                //have robot drive forward until 4 inches away from wall
                moveForward(0.30);
                if (range.getDistance(DistanceUnit.INCH) <= 4)
                    state = 1000;
                break;

            case 14:
                stateName = "Drive right until white tape is seen";
                //have robot drive right until white tape is seen
                moveRight(0.40);
                if (range.getLightDetected() >= WHITE_THRESHOLD)
                    state = 1000;
                break;

            case 16:
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