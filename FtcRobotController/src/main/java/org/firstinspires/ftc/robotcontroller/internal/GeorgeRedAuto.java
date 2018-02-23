package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Alex on 11/8/2017.
 * Autonomous Objectives:
 * --Knock Off Jewel for 30 Points
 * --Score 1 Glyph into 1st Column of CryptoBox for 15 points
 * Pseudocode:
 * 1. Knock off jewel (and read Pictograph, and lift Glyph 2 inches up)
 * 2. Drive backward to drive off balancing stone
 * 3. Drive forward to align with balancing stone
 * 4. Drive backward slowly to position needed to score in correct column
 * 5. Rotate to -75 degrees to have glyph face CryptoBox
 * 6. Drive forward toward CryptoBox until glyph is scored
 * 7. Release the glyph
 * 8. Drive backward a little bit to park
 * End. Robot ends up aligned to score glyph in specific column of CryptoBox
 */
@Autonomous(name = "GeorgeRedAuto", group = "default")
public class GeorgeRedAuto extends GeorgeOp {
    //Declare and Initialize any variables needed for this specific autonomous program

    public GeorgeRedAuto() {}

    @Override
    public void init() {
        //Initialize motors & set direction
        driveFR = hardwareMap.dcMotor.get("dfr");
        driveFR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveFL = hardwareMap.dcMotor.get("dfl");
        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBR = hardwareMap.dcMotor.get("dbr");
        driveBR.setDirection(DcMotorSimple.Direction.FORWARD);
        driveBL = hardwareMap.dcMotor.get("dbl");
        driveBL.setDirection(DcMotorSimple.Direction.REVERSE);
        glyphLift = hardwareMap.dcMotor.get("gl");
        glyphLift.setDirection(DcMotorSimple.Direction.REVERSE);
        relicMotor = hardwareMap.dcMotor.get("rm");
        relicMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Initialize servos
        upDownServo = hardwareMap.servo.get("uds");
        leftRightServo = hardwareMap.servo.get("lrs");
        leftClaw = hardwareMap.servo.get("lc");
        rightClaw = hardwareMap.servo.get("rc");
        topLeftClaw = hardwareMap.servo.get("lct");
        topRightClaw = hardwareMap.servo.get("rct");

        //Initialize sensors
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("cs");
        colorSensor.enableLed(true);
        driveVoltage = hardwareMap.voltageSensor.get("Expansion Hub 2");
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameterz.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);

        //Initialize Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = APIKey.apiKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // Use FRONT Camera (Change to BACK if you want to use that one)
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }

    @Override public void start() {
        relicTrackables.activate();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateName, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));
        if (pictographKey == 0) {
            telemetry.addData("Pictograph", "LEFT");
        }
        else if (pictographKey == 1) {
            telemetry.addData("Pictograph", "MIDDLE");
        }
        else if (pictographKey == 2) {
            telemetry.addData("Pictograph", "RIGHT");
        }
        else if (pictographKey == -1) {
            telemetry.addData("Pictograph", "COMPLETE");
        }

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
                //Check Pictograph to score glyph in correct column
                updateVuforia();
                //Close the claw servos to grab the glyph
                leftClawServoPos = SERVO_GRAB_LEFT;
                leftClaw.setPosition(leftClawServoPos);
                rightClawServoPos = SERVO_GRAB_RIGHT;
                rightClaw.setPosition(rightClawServoPos);
                leftClawTopServoPos = SERVO_GRAB_LEFT_TOP;
                topLeftClaw.setPosition(leftClawTopServoPos);
                rightClawTopServoPos = SERVO_GRAB_RIGHT_TOP;
                topRightClaw.setPosition(rightClawTopServoPos);
                //upDownServo moves down to down position
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
                //Check Pictograph to score glyph in correct column
                updateVuforia();

                //if leftJewel == red, leftRightServo moves left to knock off blue jewel
                //if leftJewel == blue, leftRightServo moves right to knock off blue jewel
                colorSensor.enableLed(true);//Turns Color Sensor into Active Mode
                if (colorSensor.red() >= RED_THRESHOLD) {
                    leftRightPos = LEFTRIGHT_MAX;
                    jewelTime = this.time;
                    jewelKnocked = true;
                }
                else if (colorSensor.blue() >= BLUE_THRESHOLD) {
                    leftRightPos = LEFTRIGHT_MIN;
                    jewelTime = this.time;
                    jewelKnocked = true;
                }
                else if (waitSec(1) && !jewelKnocked) //Fail Safe: If looking into hole
                    leftRightPos -= 0.0025;
                leftRightPos = Range.clip(leftRightPos, LEFTRIGHT_MIN, LEFTRIGHT_MAX);
                leftRightServo.setPosition(leftRightPos);
                if (waitJewelSec(0.5) && (leftRightServo.getPosition() == LEFTRIGHT_MAX || leftRightServo.getPosition() == LEFTRIGHT_MIN)) {
                    state++;
                    glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                break;

            case 6:
                stateName = "Knock off jewel 3 - arm up";
                //Check Pictograph to score glyph in correct column
                updateVuforia();
                if (!waitSec(2.0)) {//bring up glyph
                    glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    glyphLift.setPower(0.30);
                }
                else
                    glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //leftRight servo moves back to center of robot
                if (leftRightPos != LEFTRIGHT_MID)
                    leftRightPos = LEFTRIGHT_MID;
                leftRightServo.setPosition(leftRightPos);

                //upDownServo moves back to up position
                upDownPos += 0.03;
                upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
                upDownServo.setPosition(upDownPos);
                if (upDownServo.getPosition() == UPDOWN_MAX && waitSec(2.5))
                    state++;
                break;

            case 8:
                stateName = "Drive backward to drive off balancing stone";
                //Check Pictograph to score glyph in correct column
                updateVuforia();
                moveForward(-0.20, -1.5);
                if (encoderTargetReached)
                    state++;
                break;

            case 10:
                stateName = "Drive forward to align with balancing stone";
                //have robot drive to be square with the balancing stone
                moveForward(0.20);
                if (waitSec(0.5))
                    state++;
                break;

            case 12:
                stateName = "Drive Back until correct column reached";
                if (pictographKey == 2) { //drive to right column
                    moveForward(-0.20, -1.30);
                }
                else if (pictographKey == 1) { //drive to middle column
                    moveForward(-0.20, -2.100);
                }
                else if (pictographKey == 0) { //drive to left column
                    moveForward(-0.20, -2.325);
                }
                if (encoderTargetReached) {
                    if (pictographKey == 0) {
                        state++;
                        pictographKey = -2; //tells robot to robot to different angle
                    }
                    else {
                        state++;
                        pictographKey = -1;
                    }
                }
                break;

            case 14:
                stateName = "Rotate to -85 degrees to have glyph face CryptoBox";
                //if cyrptokey = left column rotate to -85 degrees to prevent from hitting balancing stone
                if (pictographKey == -2) {
                    turnClockwise(-85);
                    if (turnAbsolute(-85)) {
                        state++;
                        pictographKey = -1;
                    }
                }
                else {
                    turnClockwise(-75);
                    if (turnAbsolute(-75))
                        state++;
                }
                break;

            case 16:
                stateName = "Drive forward toward CryptoBox until glyph is scored";
                moveForward(0.20);
                if (waitSec(3.0))
                    state++;
                break;

            case 18:
                stateName = "Drop and Release Glyph";
                //open the claw to release glyph
                leftClawServoPos = SERVO_MIN_LEFT;
                leftClaw.setPosition(leftClawServoPos);
                rightClawServoPos = SERVO_MAX_RIGHT;
                rightClaw.setPosition(rightClawServoPos);
                leftClawTopServoPos = SERVO_MIN_LEFT_TOP;
                topLeftClaw.setPosition(leftClawTopServoPos);
                rightClawTopServoPos = SERVO_MAX_RIGHT_TOP;
                topRightClaw.setPosition(rightClawTopServoPos);
                if (leftClaw.getPosition() == SERVO_MIN_LEFT && rightClaw.getPosition() == SERVO_MAX_RIGHT)
                    state++;
                break;

            case 20:
                stateName = "Drive backward a little bit to park";
                moveForward(-0.20);
                if (!waitSec(0.95)) {//bring up the glyph
                    glyphLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    glyphLift.setPower(-0.50);
                }
                else
                    glyphLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                if (waitSec(1))
                    state = 1000;
                break;

//            case 22:
//                stateName = "Drive forward to push glyph in";
//                moveForward(0.30);
//                if (waitSec(2))
//                    state++;
//                break;
//
//            case 24:
//                stateName = "Drive backward a little bit to park";
//                moveForward(-0.20);
//                if (waitSec(1.25))
//                    state = 1000;
//                break;

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
                if (waitSec(0.25)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}