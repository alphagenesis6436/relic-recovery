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
 * ----Modified Code to account for replacing downUpServo from a continuous to a 180 servo
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
    ModernRoboticsI2cGyro gyroMR; //For Mecanum Drive Train
    ModernRoboticsI2cRangeSensor range; //for detecting the wall in autonomous
    Servo leftClaw; //180, glyph claw
    Servo rightClaw; //180, glyph claw
    Servo topLeftClaw; //180, glyph top claw
    Servo topRightClaw; //180, glyph top claw
    DcMotor glyphLift; //Andymark 60:1, lift the glyph claw
    Servo downUpServo;    //180, relic claw
    Servo openCloseServo; //180, relic claw
    DcMotor relicMotor;     //40:1, relic lift

    //Mecanum Drive Train Variables and Constants
    final double DRIVE_PWR_MAX = 0.80;
    final double TURN_PWR_MAX = 0.70;
    final int COUNTS_PER_REVOLUTION = 1120; //AndyMark Motors
    final double DRIVE_GEAR_RATIO = 16.0 / 16.0; //Driven / Driver
    final double COUNTS_PER_INCH_RF = COUNTS_PER_REVOLUTION / (4 * Math.PI / DRIVE_GEAR_RATIO); //forward / right / backward / left
    final double COUNTS_PER_INCH_DG = COUNTS_PER_REVOLUTION / (2 * Math.PI * Math.sqrt(2) / DRIVE_GEAR_RATIO); //diagonal
    final int WHITE_THRESHOLD = 30;
    boolean whitePreviouslyDetected = false;
    boolean drivePreciseIsOn = false; //true halfs drive speed, false returns drive speed to max
    double forwardRightPower = 0;
    double forwardLeftPower = 0;
    double backwardRightPower = 0;
    double backwardLeftPower = 0;

    //Glyph Claw Mechanism Variables and Constants
    final float GLYPH_LIFT_PWR_MAX = 0.50f;
    double glyphLiftPower = 0;
    final float SERVO_MIN_LEFT = 142 / 255.0f; //left claw is fully open
    final float SERVO_MID_LEFT = 186 / 255.0f; //left claw is slightly open
    final float SERVO_GRAB_LEFT = 211 / 255.0f; //left claw is gripping glyph
    final float SERVO_GRAB_RIGHT = 95 / 255.0f; //right claw is gripping the glyph 168
    final float SERVO_MID_RIGHT = 121 / 255.0f; //right claw is slightly open 188
    final float SERVO_MAX_RIGHT = 133 / 255.0f; //right claw is fully open 203
    double leftClawServoPos = SERVO_MIN_LEFT; //start left claw fully open
    double rightClawServoPos = SERVO_MAX_RIGHT; //start right claw fully open
    final float SERVO_MIN_LEFT_TOP = 173 / 255.0f; //left claw is fully open
    final float SERVO_MID_LEFT_TOP = 207 / 255.0f; //left claw is slightly open
    final float SERVO_GRAB_LEFT_TOP = 240 / 255.0f; //left claw is gripping glyph
    final float SERVO_GRAB_RIGHT_TOP = 102 / 255.0f; //right claw is gripping the glyph
    final float SERVO_MID_RIGHT_TOP = 143 / 255.0f; //right claw is slightly open
    final float SERVO_MAX_RIGHT_TOP = 163 / 255.0f; //right claw is fully open
    double leftClawTopServoPos = SERVO_MIN_LEFT_TOP; //start left claw fully open
    double rightClawTopServoPos = SERVO_MAX_RIGHT_TOP; //start right claw fully open
    boolean singleClawModeIsOn = false;
    double clawDelta = 0.0075;

    int currentLevel = 0; //start off at currentLevel 0
    int zeroLevelHeight = 10; //encoder count at currentLevel 0
    int firstLevelHeight = 1010; //encoder count at currentLevel 1
    int secondLevelHeight = 2010; //encoder count at currentLevel 2
    int thirdLevelHeight = 3010; //encoder count at currentLevel 3
    final int LEVEL_MIN = 0;
    final int LEVEL_MAX = 3;

    //Jewel Mechanism Variables and Constants
    final float LEFTRIGHT_MID = 110 / 255.0f;
    final float UPDOWN_MIN = 130 / 255.0f;   //fully down
    final float UPDOWN_MAX = 207 / 255.0f;  //fully up
    final float LEFTRIGHT_MIN = 70 / 255.0f; //far right
    final float LEFTRIGHT_MAX = 140 / 255.0f;   //far left
    final int BLUE_THRESHOLD = 3;   //holes
    final int RED_THRESHOLD = 3;    //holes
    double upDownPos = UPDOWN_MAX;
    double leftRightPos = LEFTRIGHT_MID;
    double jewelDelta = 0.01;

    //Relic Mechanism Variables and Constants
    final float OC_SERVO_MIN = 39 / 255.0f;
    final float OC_SERVO_OPEN = 200 / 255.0f; //open
    final float OC_SERVO_MAX = 255 / 255.0f; //closed
    final double DU_SERVO_MIN = 0 / 255.0f; //up - 40
    final double DU_SERVO_MAX = 107 / 255.0f; //down - 115
    double downUpServoPos = DU_SERVO_MIN;
    double openCloseServoPos = OC_SERVO_MIN;
    final double RELIC_PWR_MAX = 0.40;
    double relicPower = 0;
    double relicDelta = 0.03;

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
        downUpServo = hardwareMap.servo.get("du");
        openCloseServo = hardwareMap.servo.get("oc");

        //Initialize sensors
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("cs");
        gyroMR = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gs");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
        colorSensor.enableLed(true);

        //Initialize Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = APIKey.apiKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT; // Use FRONT Camera (Change to BACK if you want to use that one)
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        telemetry();
    }
    @Override public void start() {
        relicTrackables.activate();
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
        //updateJewel();
        updateRelic();
        //updateVuforia();
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
        leftClawServoPos = Range.clip(leftClawServoPos, SERVO_MIN_LEFT, SERVO_GRAB_LEFT);
        leftClaw.setPosition(leftClawServoPos);
        rightClawServoPos = Range.clip(rightClawServoPos, SERVO_GRAB_RIGHT, SERVO_MAX_RIGHT);
        rightClaw.setPosition(rightClawServoPos);
        glyphLiftPower = Range.clip(glyphLiftPower, -GLYPH_LIFT_PWR_MAX, GLYPH_LIFT_PWR_MAX);
        glyphLift.setPower(glyphLiftPower);
        leftClawTopServoPos = Range.clip(leftClawTopServoPos, SERVO_MIN_LEFT_TOP, SERVO_GRAB_LEFT_TOP);
        topLeftClaw.setPosition(leftClawTopServoPos);
        rightClawTopServoPos = Range.clip(rightClawTopServoPos, SERVO_GRAB_RIGHT_TOP, SERVO_MAX_RIGHT_TOP);
        topRightClaw.setPosition(rightClawTopServoPos);
        //Clip and Initialize Jewel Mechanism
        upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
        upDownServo.setPosition(upDownPos);
        leftRightPos = Range.clip(leftRightPos, LEFTRIGHT_MIN, LEFTRIGHT_MAX);
        leftRightServo.setPosition(leftRightPos);
        //Clip and Initialize Relic Mechanism
        downUpServoPos = Range.clip(downUpServoPos, DU_SERVO_MIN, DU_SERVO_MAX);
        downUpServo.setPosition(downUpServoPos);
        openCloseServoPos = Range.clip(openCloseServoPos, OC_SERVO_MIN, OC_SERVO_MAX);
        openCloseServo.setPosition(openCloseServoPos);
        relicPower = Range.clip(relicPower, -RELIC_PWR_MAX, RELIC_PWR_MAX);
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
        telemetry.addData("Distance", String.format("%.2f", range.getDistance(DistanceUnit.INCH)) + " in");
        telemetry.addData("LC Pos", String.format("%.0f", leftClaw.getPosition() * 255));
        telemetry.addData("RC Pos", String.format("%.0f", rightClaw.getPosition() * 255));
        telemetry.addData("TLC Pos", String.format("%.0f", topLeftClaw.getPosition() * 255));
        telemetry.addData("TRC Pos", String.format("%.0f", topRightClaw.getPosition() * 255));
        telemetry.addData("GL Pwr", String.format("%.2f", glyphLift.getPower()));
        telemetry.addData("DU Pos", String.format("%.0f", downUpServo.getPosition() * 255));
        telemetry.addData("OC Pos", String.format("%.0f", openCloseServo.getPosition() * 255));
        telemetry.addData("RM Pwr", String.format("%.2f", relicMotor.getPower()));
    }

    //Create Methods that will update the driver data
    void updateDriveTrain() {
        forwardRightPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        forwardLeftPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        backwardRightPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        backwardLeftPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        if (gamepad1.right_bumper) {
            drivePreciseIsOn = false;
            colorSensor.enableLed(false);
        }
        else if (gamepad1.left_bumper) {
            drivePreciseIsOn = true;
            colorSensor.enableLed(true);
        }
        if (drivePreciseIsOn) {
            forwardRightPower *= 0.40;
            forwardLeftPower *= 0.40;
            backwardRightPower *= 0.40;
            backwardLeftPower *= 0.40;
        }
    }
    //Controlled by Driver 2
    //Step 1: Open Left/Right Claw by pressing the Left/Right Bumper
    //Step 2: Close the Left/Right Claw by pressing the Left/Right Trigger
    void updateGlyphClaw() {
        glyphLiftPower = -gamepad2.left_stick_y * GLYPH_LIFT_PWR_MAX;
        if (gamepad2.a)
            singleClawModeIsOn = true;
        if (gamepad2.y)
            singleClawModeIsOn = false;
        if (!singleClawModeIsOn) {
            if (gamepad2.left_bumper) {
                leftClawServoPos = SERVO_MIN_LEFT; //left servo fully open
                rightClawServoPos = SERVO_MAX_RIGHT; //right servo fully open
                leftClawTopServoPos = SERVO_MIN_LEFT_TOP;//left top servo fully open
                rightClawTopServoPos = SERVO_MAX_RIGHT_TOP; //right top servo fully open
            }
            else if (gamepad2.right_bumper) {
                leftClawServoPos = SERVO_GRAB_LEFT; //left servo grabbing position
                rightClawServoPos = SERVO_GRAB_RIGHT; //right servo grabbing position
                leftClawTopServoPos = SERVO_GRAB_LEFT_TOP; //left top servo grabbing position
                rightClawTopServoPos = SERVO_GRAB_RIGHT_TOP; //right top servo grabbing position
            }
            else if (gamepad2.left_trigger >= 0.20) {
                leftClawServoPos = SERVO_MID_LEFT; //left servo slightly open
                rightClawServoPos = SERVO_MID_RIGHT; //right servo slightly open
                leftClawTopServoPos = SERVO_MID_LEFT_TOP; //left top servo slightly open
                rightClawTopServoPos = SERVO_MID_RIGHT_TOP; //right top servo slightly open
            }
        }
        else {
            if (gamepad2.right_bumper) {
                leftClawTopServoPos = SERVO_GRAB_LEFT_TOP; //left top servo grabbing position
                rightClawTopServoPos = SERVO_GRAB_RIGHT_TOP; //right top servo grabbing position
            }
            else if (gamepad2.left_bumper) {
                leftClawTopServoPos = SERVO_MIN_LEFT_TOP;//left top servo fully open
                rightClawTopServoPos = SERVO_MAX_RIGHT_TOP; //right top servo fully open
            }
            if (gamepad2.right_trigger >= 0.20) {
                leftClawServoPos = SERVO_GRAB_LEFT; //left servo grabbing position
                rightClawServoPos = SERVO_GRAB_RIGHT; //right servo grabbing position
            }
            else if (gamepad2.left_trigger >= 0.20) {
                leftClawServoPos = SERVO_MIN_LEFT; //left servo fully open
                rightClawServoPos = SERVO_MAX_RIGHT; //right servo fully open
            }
        }

        //Manually Control Glyph Lift
        glyphLiftPower = -gamepad2.left_stick_y * GLYPH_LIFT_PWR_MAX;
    }
    void updateJewel() {
        if (gamepad2.y)
            upDownPos += jewelDelta;
        else if (gamepad2.a)
            upDownPos -= jewelDelta;
        if (gamepad2.b)
            leftRightPos -= jewelDelta;
        else if (gamepad2.x)
            leftRightPos += jewelDelta;
    }

    void updateRelic() {
        relicPower = -gamepad2.right_stick_y * RELIC_PWR_MAX;
        if (gamepad2.dpad_up)
            openCloseServoPos = OC_SERVO_OPEN;
        else if (gamepad2.dpad_down)
            openCloseServoPos = OC_SERVO_MAX;
        if (gamepad2.dpad_right)
            downUpServoPos += relicDelta;
        else if (gamepad2.dpad_left)
            downUpServoPos -= relicDelta;
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
    void moveForward(double power, double revolutions) {
        double target = revolutions * COUNTS_PER_REVOLUTION;

        if (!encoderTargetReached) {
            moveForward(power);
        }

        if ((target > 0 && target - driveFR.getCurrentPosition() <= 10) || (target < 0 && -target + driveFR.getCurrentPosition() <= 10)) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {
            //Wait until target position is reached
            telemetry.addData("FR Encoder", driveFR.getCurrentPosition());
        }

    }
    void moveRight(double power) {
        runConstantSpeed();
        move(-power, power, power, -power);
    }
    void moveRight(double power, double revolutions) {
        double target = revolutions * COUNTS_PER_REVOLUTION;

        if (!encoderTargetReached) {
            moveRight(power);
        }

        if ((target > 0 && target - driveFR.getCurrentPosition() <= 10) || (target < 0 && -target + driveFR.getCurrentPosition() <= 10)) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {
            //Wait until target position is reached
            telemetry.addData("FR Encoder", driveFR.getCurrentPosition());
        }
    }
    void moveForwardRight(double power) {
        runConstantSpeed();
        move(0.0, power, power, 0.0);
    }
    void moveForwardRight(double power, double revolutions) {
        double target = revolutions * COUNTS_PER_REVOLUTION;

        if (!encoderTargetReached) {
            moveForwardRight(power);
        }

        if ((target > 0 && target - driveFL.getCurrentPosition() <= 10) || (target < 0 && -target + driveFL.getCurrentPosition() <= 10)) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {
            //Wait until target position is reached
            telemetry.addData("FL Encoder", driveFL.getCurrentPosition());
        }
    }
    void moveForwardLeft(double power) {
        runConstantSpeed();
        move(power, 0.0, 0.0, power);
    }
    void moveForwardLeft(double power, double revolutions) {
        double target = revolutions * COUNTS_PER_REVOLUTION;

        if (!encoderTargetReached)
            moveForwardLeft(power);

        if ((target > 0 && target - driveFR.getCurrentPosition() <= 10) || (target < 0 && -target + driveFR.getCurrentPosition() <= 10)) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {
            //Wait until target position is reached
            telemetry.addData("FR Encoder", driveFR.getCurrentPosition());
        }
    }
    void turnClockwise(double power) {
        runConstantSpeed();
        move(-power, power, -power, power);
    }
    void turnClockwise(int targetAngle) {
        double k = 1; //experimentally found
        double power = k * (targetAngle + gyroMR.getIntegratedZValue())
                / Math.abs(targetAngle);
        if (Math.abs(targetAngle + gyroMR.getIntegratedZValue()) >= 10)
            turnClockwise(power);
        else
            stopDriveMotors();
    }

    boolean turnAbsolute(double target) { //Tells robot to rotate to an absolute heading (degrees)
        boolean absoluteReached = false;
        if (Math.abs(gyroMR.getIntegratedZValue() + target) <= 10) {
            stopDriveMotors();
            absoluteReached = true;
        }
        return absoluteReached;
    }

    void updateVuforia() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) { // Test to see if image is visable
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose(); // Get Positional value to use later
            //telemetry.addData("Pose", format(pose));
            if (pose != null)
            {
            }
            if (vuMark == RelicRecoveryVuMark.LEFT)
            { // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                pictographKey = 0;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT)
            { // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                pictographKey = 2;
            } else if (vuMark == RelicRecoveryVuMark.CENTER)
            { // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                pictographKey = 1;
            }
        } else
        {
            telemetry.addData("VuMark", "not visible");
        }
    }

    void calibrateVariables() {//Used if any autonomous methods need initial state variables
        encoderTargetReached = false;
        colorSensor.enableLed(false);
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

