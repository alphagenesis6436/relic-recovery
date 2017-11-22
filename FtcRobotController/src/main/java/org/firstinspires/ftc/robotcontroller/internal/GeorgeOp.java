package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Updated by Alex on 11/18/17
 * GeorgeOp is the TeleOp for the Competition Robot
 * Has Following Robot Systems:
 * --Mecanum Drive Train
 * ----Follows Same Code from MecanumDriveOp
 * ----Adds Gyro Sensor for Autonomous
 * ----Adds turnAbsolute() method for autonomous
 * ----Adds calibrateVariables() method for autonomous
 * ----Modified encoder-based autonomous driving methods based on video by SwerveRobotics
 * ----Adds runToPosition() method for encoder-based driving methods
 * --Jewel Mechanism
 * ----Adapted from PrototypeJewelOpMode3
 * ----Modifies updateActuators() method [Renamed to updateJewel()] to accommodate Controls for other Systems
 * ----Adds in the Color Sensor for Autonomous
 * --Stone Mechanism
 * ----Adapted from PrototypeStoneOp
 * ----Declared and Initialized stoneServo
 * ----Created Variables to control stoneServo during teleop
 * ----Create method updateStone() to control stone mechanism
 * --Glyph Claw Mechanism
 * ----Adapted from PrototypeGlyphClawOp
 * ----Adds updateGlyphClawOp() method to control glyph claw mechanism
 * ----Declared / initialized glyphLift, leftClaw, rightClaw
 * ----Created Variables to control glyphLift, leftClaw, rightClaw during TeleOp
 * --Relic System
 * ----Adapted code from PrototypeRelicOp2
 * ----Adds updateRelic() method to control Relic System
 * --Vuforia System
 * ----Follows Same Code from VuforiaTestOp
 * ----Adds updateVuforia method for autonomous
 */
@TeleOp(name = "GeorgeOp", group = "Default")
public class GeorgeOp extends OpMode {
    //Declare any motors, servos, and sensors
    DcMotor driveFR; //AndyMark, 40:1
    DcMotor driveFL; //AndyMark, 40:1
    DcMotor driveBR; //AndyMark, 40:1
    DcMotor driveBL; //AndyMark, 40:1
    Servo upDownServo; //Metal Gear, 180
    Servo leftRightServo; //Metal Gear, 180
    ModernRoboticsI2cColorSensor colorSensor; //For Jewel Mechanism
    ModernRoboticsI2cColorSensor colorSensor2; //For Cryptobox Tape
    ModernRoboticsI2cGyro gyroMR; //For Mecanum Drive Train
    ModernRoboticsI2cRangeSensor range; //for detecting the wall in autonomous
    Servo leftClaw; //180, glyph claw
    Servo rightClaw; //180, glyph claw
    DcMotor glyphLift; //Andymark 60:1, lift the glyph claw
    Servo downUpServo;    //360, relic claw
    Servo openCloseServo; //180, relic claw
    DcMotor relicMotor;     //40:1, relic lift

    //Mecanum Drive Train Variables and Constants
    final double DRIVE_PWR_MAX = 0.70;
    final double TURN_PWR_MAX = 0.70;
    final int COUNTS_PER_REVOLUTION = 1120; //AndyMark Motors
    final double DRIVE_GEAR_RATIO = 16.0 / 24.0; //Driven / Driver
    final double COUNTS_PER_INCH_RF = COUNTS_PER_REVOLUTION / (4 * Math.PI / DRIVE_GEAR_RATIO); //forward / right / backward / left
    final double COUNTS_PER_INCH_DG = COUNTS_PER_REVOLUTION / (2 * Math.PI * Math.sqrt(2) / DRIVE_GEAR_RATIO); //diagonal
    final int WHITE_THRESHOLD = 30;
    boolean whitePreviouslyDetected = false;
    double forwardRightPower = 0;
    double forwardLeftPower = 0;
    double backwardRightPower = 0;
    double backwardLeftPower = 0;

    //Glyph Claw Mechanism Variables and Constants
    final float GLYPH_LIFT_PWR_MAX = 0.40f; //experimentally found by using LabView
    double glyphLiftPower = 0;
    final float SERVO_MIN_LEFT = 50 / 255.0f; //left claw is closed?
    final float SERVO_GRAB_LEFT = 80 / 255.0f; //left claw is gripping glyph
    final float SERVO_MAX_LEFT = 117 / 255.0f; //left claw is open
    final float SERVO_MIN_RIGHT = 90 / 255.0f; //right claw is open
    final float SERVO_GRAB_RIGHT = 127 / 255.0f; //right claw is gripping glpyh
    final float SERVO_MAX_RIGHT = 245 / 255.0f; //right claw is closed?
    double leftClawServoPos = SERVO_MIN_LEFT;
    double rightClawServoPos = SERVO_MAX_RIGHT;
    int currentLevel = 0; //start off at currentLevel 0
    int zeroLevelHeight = 10; //encoder count at currentLevel 0
    int firstLevelHeight = 1010; //encoder count at currentLevel 1
    int secondLevelHeight = 2010; //encoder count at currentLevel 2
    int thirdLevelHeight = 3010; //encoder count at currentLevel 3
    final int LEVEL_MIN = 0;
    final int LEVEL_MAX = 3;

    //Jewel Mechanism Variables and Constants
    final float LEFTRIGHT_MID = 110 / 255.0f;
    final float UPDOWN_MIN = 65 / 255.0f;   //fully down
    final float UPDOWN_MAX = 229
            / 255.0f;  //fully up
    final float LEFTRIGHT_MIN = 70 / 255.0f; //far right
    final float LEFTRIGHT_MAX = 140 / 255.0f;   //far left
    final int BLUE_THRESHOLD = 3;   //holes
    final int RED_THRESHOLD = 3;    //holes
    double upDownPos = UPDOWN_MAX;
    double leftRightPos = LEFTRIGHT_MID;
    double jewelDelta = 0.01;

    //Relic Mechanism Variables and Constants
    final float OC_SERVO_MIN = 160 / 255.0f; //open
    final float OC_SERVO_MAX = 255 / 255.0f; //closed
    final double DU_MAX_SPEED = (1.00) / 2.0;
    double downUpServoSpeed = 0.50;
    double openCloseServoPos = OC_SERVO_MIN;
    double RELIC_PWR_MAX = 0.40;
    double relicPower = 0;
    double relicDelta = 0.05;

    //Vuforia System Variables and Objects
    //Declare any objects for Vuforia
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    int pictographKey = 0; //Left = 0, Center = 1, Right = 2

    public GeorgeOp() {}

    @Override public void init() {
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
        glyphLift.setDirection(DcMotorSimple.Direction.REVERSE); //may need to be revised
        relicMotor = hardwareMap.dcMotor.get("rm");
        relicMotor.setDirection(DcMotorSimple.Direction.FORWARD);//may be revised

        //Initialize servos
        upDownServo = hardwareMap.servo.get("uds");
        leftRightServo = hardwareMap.servo.get("lrs");
        leftClaw = hardwareMap.servo.get("lc");
        rightClaw = hardwareMap.servo.get("rc");
        downUpServo = hardwareMap.servo.get("du");
        openCloseServo = hardwareMap.servo.get("oc");

        //Initialize sensors
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("cs");
        colorSensor2 = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("cs2");
        gyroMR = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gs");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
        colorSensor.enableLed(true);
        colorSensor2.enableLed(true);

        //Initialize Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = APIKey.apiKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // Use FRONT Camera (Change to BACK if you want to use that one)
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

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
        updateGlyphClaw();
        updateJewel();
        updateRelic();
    }

    void initialization() {
        //Clip and Initialize Drive Train
        forwardRightPower = Range.clip(forwardRightPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveFR.setPower(forwardRightPower);
        forwardLeftPower = Range.clip(forwardLeftPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveFL.setPower(forwardLeftPower);
        backwardRightPower = Range.clip(backwardRightPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveBR.setPower(backwardRightPower);
        backwardLeftPower = Range.clip(backwardLeftPower, -DRIVE_PWR_MAX, DRIVE_PWR_MAX);
        driveBL.setPower(backwardLeftPower);
        //Clip and Initialize Glyph Claw Mechanism
        leftClawServoPos = Range.clip(leftClawServoPos, SERVO_MIN_LEFT, SERVO_MAX_LEFT);
        leftClaw.setPosition(leftClawServoPos);
        rightClawServoPos = Range.clip(rightClawServoPos, SERVO_MIN_RIGHT, SERVO_MAX_RIGHT);
        rightClaw.setPosition(rightClawServoPos);
        glyphLiftPower = Range.clip(glyphLiftPower, -0.05, GLYPH_LIFT_PWR_MAX);
        glyphLift.setPower(glyphLiftPower);
        //Clip and Initialize Jewel Mechanism
        upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
        upDownServo.setPosition(upDownPos);
        leftRightPos = Range.clip(leftRightPos, LEFTRIGHT_MIN, LEFTRIGHT_MAX);
        leftRightServo.setPosition(leftRightPos);
        //Clip and Initialize Relic Mechanism
        downUpServoSpeed = Range.clip(downUpServoSpeed, -1, 1);
        downUpServo.setPosition(downUpServoSpeed);
        openCloseServoPos = Range.clip(openCloseServoPos, OC_SERVO_MIN, OC_SERVO_MAX);
        openCloseServo.setPosition(openCloseServoPos);
        relicPower = Range.clip(relicPower, -0.10, RELIC_PWR_MAX);
        relicMotor.setPower(relicPower);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("FR Pwr", String.format("%.2f",driveFR.getPower()));
        telemetry.addData("FL Pwr", String.format("%.2f",driveFL.getPower()));
        telemetry.addData("BR Pwr", String.format("%.2f",driveBR.getPower()));
        telemetry.addData("BL Pwr", String.format("%.2f",driveBL.getPower()));
        telemetry.addData("UD Pos", String.format("%.0f", upDownServo.getPosition() * 255));
        telemetry.addData("LR Pos", String.format("%.0f", leftRightServo.getPosition() * 255));
        telemetry.addData("Gyro", gyroMR.getIntegratedZValue());
        telemetry.addData("Red1", colorSensor.red());
        telemetry.addData("Blue1", colorSensor.blue());
        telemetry.addData("Red2", colorSensor2.red());
        telemetry.addData("Blue2", colorSensor2.blue());
        telemetry.addData("Green2", colorSensor2.green());
        telemetry.addData("Distance", String.format("%.2f", range.getDistance(DistanceUnit.INCH)) + " in");
        telemetry.addData("LC Pos", String.format("%.0f", leftClaw.getPosition() * 255));
        telemetry.addData("RC Pos", String.format("%.0f", rightClaw.getPosition() * 255));
        telemetry.addData("GL Pwr", String.format("%.2f", glyphLift.getPower()));
        telemetry.addData("DU Speed", String.format("%.0f", downUpServo.getPosition()));
        telemetry.addData("OC Pos", String.format("%.0f", openCloseServo.getPosition() * 255));
        telemetry.addData("RM Pwr", String.format("%.2f", relicMotor.getPower()));
    }

    //Create Methods that will update the driver data
    void updateDriveTrain() {
        forwardRightPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        forwardLeftPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        backwardRightPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        backwardLeftPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
    }
    //Controlled by Driver 2
    //Step 1: Open Left/Right Claw by pressing the Left/Right Bumper
    //Step 2: Close the Left/Right Claw by pressing the Left/Right Trigger
    void updateGlyphClaw() {
        glyphLiftPower = -gamepad2.left_stick_y * GLYPH_LIFT_PWR_MAX;
        if (gamepad2.left_bumper) {
            leftClawServoPos = SERVO_MAX_LEFT; //left servo open
            rightClawServoPos = SERVO_MIN_RIGHT; //right servo open
        }
        if (gamepad2.right_bumper) {
            leftClawServoPos = SERVO_GRAB_LEFT;
            rightClawServoPos = SERVO_GRAB_RIGHT;
        }

        //Manually Control Glyph Lift
        glyphLiftPower = -gamepad2.left_stick_y * 0.50;

        /*Elevator lift
        * glyph claw should only exist at certain heights / "levels" (similar to Bohr's energy levels)
        * each currentLevel corresponds to one of 4 heights at which blocks can be dropped
        */

        //relies on encoder counts
        /*switch (currentLevel) {
            case 0:
                //bring elevator down to zero position
                if (glyphLift.getCurrentPosition() - zeroLevelHeight >= 10) {
                    glyphLift.setTargetPosition(zeroLevelHeight);
                    glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    glyphLift.setPower(-GLYPH_LIFT_PWR_MAX);
                }
                //bring elevator up to zero position
                else if (zeroLevelHeight - glyphLift.getCurrentPosition() >= 10) {
                    glyphLift.setTargetPosition(zeroLevelHeight);
                    glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    glyphLift.setPower(GLYPH_LIFT_PWR_MAX);
                }
                break;
            case 1:
                //bring elevator down to 1st block height
                if (glyphLift.getCurrentPosition() - firstLevelHeight >= 10) {
                    glyphLift.setTargetPosition(firstLevelHeight);
                    glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    glyphLift.setPower(-GLYPH_LIFT_PWR_MAX);
                }
                //bring elevator up to 1st block height
                else if (firstLevelHeight - glyphLift.getCurrentPosition() >= 10) {
                    glyphLift.setTargetPosition(firstLevelHeight);
                    glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    glyphLift.setPower(GLYPH_LIFT_PWR_MAX);
                }
                break;
            case 2:
                //bring elevator down to 2nd block height
                if (glyphLift.getCurrentPosition() - secondLevelHeight >= 10) {
                    glyphLift.setTargetPosition(secondLevelHeight);
                    glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    glyphLift.setPower(-GLYPH_LIFT_PWR_MAX);
                }
                //bring elevator up to 2nd block height
                else if (secondLevelHeight - glyphLift.getCurrentPosition() >= 10) {
                    glyphLift.setTargetPosition(secondLevelHeight);
                    glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    glyphLift.setPower(GLYPH_LIFT_PWR_MAX);
                }
                break;
            case 3:
                //bring elevator down to 3rd block height
                if (glyphLift.getCurrentPosition() - thirdLevelHeight >= 10) {
                    glyphLift.setTargetPosition(thirdLevelHeight);
                    glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    glyphLift.setPower(-GLYPH_LIFT_PWR_MAX);
                }
                //bring elevator up to 3rd block height
                else if (thirdLevelHeight - glyphLift.getCurrentPosition() >= 10) {
                    glyphLift.setTargetPosition(thirdLevelHeight);
                    glyphLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    glyphLift.setPower(GLYPH_LIFT_PWR_MAX);
                }
                break;
        }

        if (gamepad2.y && currentLevel < LEVEL_MAX)
            currentLevel++;
        else if (gamepad2.a && currentLevel > LEVEL_MIN)
            currentLevel--;*/
    }
    void updateJewel() {
//        if (gamepad2.dpad_up)
//            upDownPos += jewelDelta;
//        else if (gamepad2.dpad_down)
//            upDownPos -= jewelDelta;
//        if (gamepad2.dpad_right)
//            leftRightPos -= jewelDelta;
//        else if (gamepad2.dpad_left)
//            leftRightPos += jewelDelta;
    }

    void updateRelic() {
        downUpServoSpeed = 0.50;
        downUpServoSpeed += gamepad2.right_trigger * DU_MAX_SPEED;
        downUpServoSpeed -= gamepad2.left_trigger * DU_MAX_SPEED;
        relicPower = -gamepad2.right_stick_y * RELIC_PWR_MAX;
        if (gamepad2.dpad_up)
            openCloseServoPos -= relicDelta;
        else if (gamepad2.dpad_down)
            openCloseServoPos += relicDelta;
    }


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous
    boolean encoderTargetReached = false;

    void resetEncoders() {
        driveFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    void runConstantSpeed() {
        driveFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void runConstantPower() {
        driveFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void runToPosition() {
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void move(double pwr_fr, double pwr_fl, double pwr_br, double pwr_bl) {
        driveFR.setPower(pwr_fr);
        driveFL.setPower(pwr_fl);
        driveBR.setPower(pwr_br);
        driveBL.setPower(pwr_bl);
    }
    void stopDriveMotors() {
        move(0, 0, 0, 0);
    }
    void moveForward(double power) {
        runConstantSpeed();
        move(power, power, power, power);
    }
    void moveForward(double power, int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_RF);

        driveFR.setTargetPosition(target);
        driveFL.setTargetPosition(target);
        driveBR.setTargetPosition(target);
        driveBL.setTargetPosition(target);
        runToPosition();
        move(power, power, power, power);

        while (driveFR.isBusy()) {
            //Wait until target position is reached
            telemetry.addData("FR Encoder", String.format("%.0f", (target - driveFR.getCurrentPosition()) / COUNTS_PER_INCH_RF));
        }
        stopDriveMotors();
        encoderTargetReached = true;
    }
    void moveRight(double power) {
        runConstantSpeed();
        move(-power, power, power, -power);
    }
    void moveRight(double power, int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_RF);

        driveFR.setTargetPosition(-target);
        driveFL.setTargetPosition(target);
        driveBR.setTargetPosition(target);
        driveBL.setTargetPosition(-target);
        runToPosition();
        move(-power, power, power, -power);

        while (driveFR.isBusy()) {
            //Wait until target position is reached
            telemetry.addData("FR Encoder", String.format("%.0f", (target - driveFR.getCurrentPosition()) / COUNTS_PER_INCH_RF));
        }
        stopDriveMotors();
        encoderTargetReached = true;
    }
    void moveForwardRight(double power) {
        runConstantSpeed();
        move(0.0, power, power, 0.0);
    }
    void moveForwardRight(double power, int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_DG);

        driveFR.setTargetPosition(0); //motor will not rotate because the motor position resets to 0 at end of each stage
        driveFL.setTargetPosition(target);
        driveBR.setTargetPosition(target);
        driveBL.setTargetPosition(0);
        runToPosition();
        move(0.0, power, power, 0.0);

        while (driveFL.isBusy()) {
            //Wait until target position is reached
            telemetry.addData("FL Encoder", String.format("%.0f", (target - driveFL.getCurrentPosition()) / COUNTS_PER_INCH_RF));
        }
        stopDriveMotors();
        encoderTargetReached = true;
    }
    void moveForwardLeft(double power) {
        runConstantSpeed();
        move(power, 0.0, 0.0, power);
    }
    void moveForwardLeft(double power, int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_DG);

        driveFR.setTargetPosition(target);
        driveFL.setTargetPosition(0);
        driveBR.setTargetPosition(0);
        driveBL.setTargetPosition(target);
        runToPosition();
        move(power, 0.0, 0.0, power);

        while (driveFR.isBusy()) {
            //Wait until target position is reached
            telemetry.addData("FR Encoder", String.format("%.0f", (target - driveFR.getCurrentPosition()) / COUNTS_PER_INCH_RF));
        }
        stopDriveMotors();
        encoderTargetReached = true;
    }
    void turnClockwise(double power) {
        runConstantSpeed();
        move(-power, power, -power, power);
    }
    void turnClockwise(int targetAngle) {
        double k = 1; //experimentally found
        double power = k * (targetAngle + gyroMR.getIntegratedZValue())
                / Math.abs(targetAngle);
        if (Math.abs(targetAngle + gyroMR.getIntegratedZValue()) >= 5)
            turnClockwise(power);
        else
            stopDriveMotors();
    }

    boolean turnAbsolute(double target) { //Tells robot to rotate to an absolute heading (degrees)
        boolean absoluteReached = false;
        if (Math.abs(gyroMR.getIntegratedZValue() + target) <= 5) {
            stopDriveMotors();
            absoluteReached = true;
        }
        return absoluteReached;
    }

    void updateVuforia() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
            if (vuMark == RelicRecoveryVuMark.LEFT)
            { // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                pictographKey = 0;
            } else if (vuMark == RelicRecoveryVuMark.CENTER)
            { // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                pictographKey = 1;
            }
            else if (vuMark == RelicRecoveryVuMark.RIGHT)
            { // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                pictographKey = 2;
            }
        } else
        {
            telemetry.addData("VuMark", "not visible");
        }
    }

    boolean whiteTapeIsDetected() {
        return (colorSensor2.red() >= WHITE_THRESHOLD && colorSensor2.blue() >= WHITE_THRESHOLD
                && colorSensor2.green() >= WHITE_THRESHOLD && !whitePreviouslyDetected);
    }
    boolean whiteTapeIsNotDetected() {
        return (!(colorSensor2.red() >= WHITE_THRESHOLD && colorSensor2.blue() >= WHITE_THRESHOLD
                && colorSensor2.green() >= WHITE_THRESHOLD) && whitePreviouslyDetected);
    }

    void calibrateVariables() {//Used if any autonomous methods need initial state variables
        encoderTargetReached = false;
        colorSensor.enableLed(false);
        colorSensor2.enableLed(false);
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

