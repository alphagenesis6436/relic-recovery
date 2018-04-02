package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;

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
    BNO055IMU imu; //For detecting rotation
    Orientation angles;
    Servo leftClaw; //180, glyph claw
    Servo rightClaw; //180, glyph claw
    Servo topLeftClaw; //180, glyph top claw
    Servo topRightClaw; //180, glyph top claw
    Servo pivotServo; //2880 winch servo, glyph claw pivot
    DcMotor glyphLift; //Andymark 60:1, lift the glyph claw
    Servo downUpServo;    //180, relic claw
    Servo openCloseServo; //180, relic claw
    DcMotor relicMotor;     //40:1, relic lift

    //Mecanum Drive Train Variables and Constants
    final double DRIVE_PWR_MAX = 0.50;
    final double TURN_PWR_MAX = 1.00;



    final int COUNTS_PER_REVOLUTION = 1120; //AndyMark Motors
    final double DRIVE_GEAR_RATIO = 16.0 / 24.0; //Driven / Driver
    final double COUNTS_PER_INCH_RF = COUNTS_PER_REVOLUTION / (4 * Math.PI / DRIVE_GEAR_RATIO); //forward / right / backward / left
    final double COUNTS_PER_INCH_DG = COUNTS_PER_REVOLUTION / (2 * Math.PI * Math.sqrt(2) / DRIVE_GEAR_RATIO); //diagonal
    boolean drivePreciseIsOn = false; //true halfs drive speed, false returns drive speed to max
    double forwardRightPower = 0;
    double forwardLeftPower = 0;
    double backwardRightPower = 0;
    double backwardLeftPower = 0;

    //Glyph Claw Mechanism Variables and Constants
    final float GLYPH_LIFT_PWR_MAX = 0.90f;
    double glyphLiftPower = 0;
    final float PIVOT_MIN = 0 / 255.0f; // (189/2295) starting position???
    final float PIVOT_MAX = 170 / 225.0f; // (365/2295) rotated 180 degrees???
    final float SERVO_GRAB_LEFT = 110 / 255.0f; //left claw is fully open
    final float SERVO_MID_LEFT = 170 / 255.0f; //left claw is slightly open
    final float SERVO_MAX_LEFT = 255 / 255.0f; //left claw is gripping glyph
    final float SERVO_MIN_RIGHT = 37 / 255.0f; //right claw is gripping the glyph 168
    final float SERVO_MID_RIGHT = 112 / 255.0f; //right claw is slightly open 188
    final float SERVO_GRAB_RIGHT = 180 / 255.0f; //right claw is fully open 203
    double leftClawServoPos = SERVO_MAX_LEFT; //start left claw fully open
    double rightClawServoPos = SERVO_MIN_RIGHT; //start right claw fully open
    final float SERVO_MIN_LEFT_TOP = 88 / 255.0f; //left claw is fully open
    final float SERVO_MID_LEFT_TOP = 175 / 255.0f; //left claw is slightly open
    final float SERVO_GRAB_LEFT_TOP = 235 / 255.0f; //left claw is gripping glyph
    final float SERVO_GRAB_RIGHT_TOP = 78 / 255.0f; //right claw is gripping the glyph
    final float SERVO_MID_RIGHT_TOP = 140 / 255.0f; //right claw is slightly open
    final float SERVO_MAX_RIGHT_TOP = 226 / 255.0f; //right claw is fully open
    double leftClawTopServoPos = SERVO_MIN_LEFT_TOP; //start left claw fully open
    double rightClawTopServoPos = SERVO_MAX_RIGHT_TOP; //start right claw fully open
    double pivotPos = PIVOT_MIN;
    boolean singleClawModeIsOn = false;
    boolean clawIsUpRight = true;
    boolean glpyhBtnPressed = false;
    double clawDelta = 0.0075;
    double pivotDelta = 1 / 225.0f; //rotated 180 degrees???

    //Jewel Mechanism Variables and Constants
    final float LEFTRIGHT_MID = 110 / 255.0f;
    final float UPDOWN_MIN = 127 / 255.0f;   //fully down (maybe 131)
    final float UPDOWN_MAX = 207 / 255.0f;  //fully up
    final float LEFTRIGHT_MIN = 50 / 255.0f; //far right (70)
    final float LEFTRIGHT_MAX = 160 / 255.0f;   //far left (140)
    final int BLUE_THRESHOLD = 3;   //holes
    final int RED_THRESHOLD = 3;    //holes
    double upDownPos = UPDOWN_MAX;
    double leftRightPos = LEFTRIGHT_MID;
    double jewelDelta = 0.01;
    boolean jewelKnocked = false;

    //Relic Mechanism Variables and Constants
    final float OC_SERVO_CLOSE = 130 / 255.0f; //close - 180 o.g.
    final float OC_SERVO_MAX = 206 / 255.0f; //open
    final double DU_SERVO_MIN = 70 / 255.0f; //up
    final double DU_SERVO_MAX = 211 / 255.0f; //down
    double downUpServoPos = DU_SERVO_MAX;
    double openCloseServoPos = OC_SERVO_CLOSE;
    final double RELIC_PWR_MAX = 0.80;
    boolean relicPwrSustained = false;
    boolean relicBtnPressed = false;
    double relicPower = 0;
    double relicDelta = 0.05;

    //Vuforia System Variables and Objects
    //Declare any objects for Vuforia
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    int pictographKey = 0; //Left = 0, Center = 1, Right = 2

    VoltageSensor driveVoltage;

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
        pivotServo = hardwareMap.servo.get("ps");

        //Initialize sensors
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("cs");
        colorSensor.enableLed(true);
        BNO055IMU.Parameters parameterz = new BNO055IMU.Parameters();
        parameterz.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterz.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterz.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameterz.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterz);
    }

    @Override public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
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
        pivotPos = Range.clip(pivotPos, PIVOT_MIN, PIVOT_MAX);
        pivotServo.setPosition(pivotPos);
        leftClawServoPos = Range.clip(leftClawServoPos, SERVO_GRAB_LEFT, SERVO_MAX_LEFT);
        leftClaw.setPosition(leftClawServoPos);
        rightClawServoPos = Range.clip(rightClawServoPos, SERVO_MIN_RIGHT, SERVO_GRAB_RIGHT);
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
        openCloseServoPos = Range.clip(openCloseServoPos, OC_SERVO_CLOSE, OC_SERVO_MAX);
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
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("IMU Heading", String.format("%.0f", angles.firstAngle));
        telemetry.addData("Red1", colorSensor.red());
        telemetry.addData("Blue1", colorSensor.blue());
        telemetry.addData("PS Pos", String.format("%.0f", pivotServo.getPosition() * 225));
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
            if (Math.round(this.time) % 2 == 0)
                colorSensor.enableLed(true);
            else
                colorSensor.enableLed(false);
        }
        if (drivePreciseIsOn) {
            runConstantSpeed();
            forwardRightPower = preciseDriveScaling(forwardRightPower);
            forwardLeftPower = preciseDriveScaling(forwardLeftPower);
            backwardRightPower = preciseDriveScaling(backwardRightPower);
            backwardLeftPower = preciseDriveScaling(backwardLeftPower);
            if (Math.round(this.time) % 2 == 0)
                colorSensor.enableLed(true);
            else
                colorSensor.enableLed(false);
        }
        else {
            colorSensor.enableLed(false);
            runConstantPower();
        }
        
    }

    double preciseDriveScaling(double v) {
        v = Math.pow(Math.tanh(v), 3)/Math.tanh(1);
        return v;
    }
    //Controlled by Driver 2
    //Step 1: Open Left/Right Claw by pressing the Left/Right Bumper
    //Step 2: Close the Left/Right Claw by pressing the Left/Right Trigger
    void updateGlyphClaw() {
        glyphLiftPower = -gamepad2.left_stick_y * GLYPH_LIFT_PWR_MAX;
        if (gamepad2.dpad_left) {
            pivotPos -= pivotDelta;
        }
        else if (gamepad2.dpad_right)
            pivotPos += pivotDelta;
        if (gamepad2.right_stick_button && !glpyhBtnPressed) {
            glpyhBtnPressed = true;
            clawIsUpRight = !clawIsUpRight;
        }
        else if (!gamepad2.right_stick_button
                && glpyhBtnPressed)
            glpyhBtnPressed = false;
        if (clawIsUpRight) {
            pivotPos = PIVOT_MIN;
            if (gamepad2.left_bumper) {
                leftClawServoPos = SERVO_MAX_LEFT; //left servo fully open
                rightClawServoPos = SERVO_MIN_RIGHT; //right servo fully open
            }
            else if (gamepad2.right_bumper) {
                leftClawServoPos = SERVO_GRAB_LEFT; //left servo grabbing position
                rightClawServoPos = SERVO_GRAB_RIGHT; //right servo grabbing position
            }
            else if (gamepad2.x) {
                leftClawServoPos = SERVO_MID_LEFT; //left servo slightly open
                rightClawServoPos = SERVO_MID_RIGHT; //right servo slightly open
            }
        }
        else {
            pivotPos = PIVOT_MAX;
            if (gamepad2.left_bumper) {
                leftClawTopServoPos = SERVO_MIN_LEFT_TOP;//left top servo fully open
                rightClawTopServoPos = SERVO_MAX_RIGHT_TOP; //right top servo fully open
            } else if (gamepad2.right_bumper) {
                leftClawTopServoPos = SERVO_GRAB_LEFT_TOP; //left top servo grabbing position
                rightClawTopServoPos = SERVO_GRAB_RIGHT_TOP; //right top servo grabbing position
            }
            else if (gamepad2.x) {
                leftClawTopServoPos = SERVO_MID_LEFT_TOP; //left top servo slightly open
                rightClawTopServoPos = SERVO_MID_RIGHT_TOP; //right top servo slightly open
            }
        }
        if (gamepad2.b) {
            leftClawServoPos = SERVO_MID_LEFT; //left servo slightly open
            rightClawServoPos = SERVO_MID_RIGHT; //right servo slightly open
            leftClawTopServoPos = SERVO_MID_LEFT_TOP; //left top servo slightly open
            rightClawTopServoPos = SERVO_MID_RIGHT_TOP; //right top servo slightly open
        }
        if (gamepad2.right_trigger >= 0.50) {
            leftClawServoPos = SERVO_GRAB_LEFT; //left servo grabbing position
            rightClawServoPos = SERVO_GRAB_RIGHT; //right servo grabbing position
            leftClawTopServoPos = SERVO_GRAB_LEFT_TOP; //left top servo grabbing position
            rightClawTopServoPos = SERVO_GRAB_RIGHT_TOP; //right top servo grabbing position
        }
        else if (gamepad2.left_trigger >= 0.50) {
            leftClawServoPos = SERVO_MAX_LEFT; //left servo fully open
            rightClawServoPos = SERVO_MIN_RIGHT; //right servo fully open
            leftClawTopServoPos = SERVO_MIN_LEFT_TOP;//left top servo fully open
            rightClawTopServoPos = SERVO_MAX_RIGHT_TOP; //right top servo fully open
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
        if (!relicPwrSustained) {
            relicPower = -gamepad2.right_stick_y * RELIC_PWR_MAX;
        }
        if (gamepad2.right_trigger >= 0.50 && !relicBtnPressed) {
            relicBtnPressed = true;
            relicPwrSustained = !relicPwrSustained;
        }
        else if (gamepad2.right_trigger < 0.30 && relicBtnPressed
                || gamepad2.back) {
            relicBtnPressed = false;
        }

        if (gamepad2.dpad_up)
            openCloseServoPos = OC_SERVO_MAX;
        else if (gamepad2.dpad_down)
            openCloseServoPos = OC_SERVO_CLOSE;
        else if (gamepad2.dpad_left)
            openCloseServoPos = (OC_SERVO_MAX + OC_SERVO_CLOSE) / 2;
        if (gamepad2.a) //down
            downUpServoPos += relicDelta;
        else if (gamepad2.y) //up
            downUpServoPos -= relicDelta;
    }


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    double jewelTime; //used to measure the time period of after hitting the jewel in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous
    boolean encoderTargetReached = false;
    boolean angleTargetReached = false;

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
    void moveForward(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION * DRIVE_GEAR_RATIO;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION;
        double error = target - driveFR.getCurrentPosition();
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION / 2) {
                speed = (0.10 * error / Math.abs(error)) + (error * kp);
            }
            moveForward(speed);
        }
        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {//Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION / DRIVE_GEAR_RATIO));
        }
    }
    void moveRight(double power) {
        runConstantSpeed();
        move(-power, power, power, -power);
    }
    void moveRight(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION * DRIVE_GEAR_RATIO;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION;
        double error = target - driveFL.getCurrentPosition();
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION / 2) {
                speed = (0.10 * revolutions / Math.abs(revolutions)) + (error * kp);
            }
            moveRight(speed);
        }

        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {
            //Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION / DRIVE_GEAR_RATIO));
        }
    }
    void moveForwardRight(double power) {
        runConstantSpeed();
        move(0.0, power, power, 0.0);
    }
    void moveForwardRight(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION * DRIVE_GEAR_RATIO;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION;
        double error = target - driveFL.getCurrentPosition();
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION / 2) {
                speed = (0.10 * revolutions / Math.abs(revolutions)) + (error * kp);
            }
            moveForwardRight(speed);
        }

        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {
            //Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION / DRIVE_GEAR_RATIO));
        }
    }
    void moveForwardLeft(double power) {
        runConstantSpeed();
        move(power, 0.0, 0.0, power);
    }
    void moveForwardLeft(double speed, double revolutions) {
        //Proportional Drive Control: for the last half rotation of the motor,
        //the motors will decelerate to from the input speed to 10% speed
        double target = revolutions * COUNTS_PER_REVOLUTION * DRIVE_GEAR_RATIO;
        double kp = 2 * (Math.abs(speed) - 0.10) / COUNTS_PER_REVOLUTION;
        double error = target - driveFR.getCurrentPosition();
        if (!encoderTargetReached) {
            if (Math.abs(error) <= COUNTS_PER_REVOLUTION / 2) {
                speed = (0.10 * revolutions / Math.abs(revolutions)) + (error * kp);
            }
            moveForwardLeft(speed);
        }

        if (Math.abs(error) <= 4) {
            stopDriveMotors();
            encoderTargetReached = true;
        }
        else {
            //Wait until target position is reached
            telemetry.addData("Rotations left", String.format("%.2f", error / COUNTS_PER_REVOLUTION / DRIVE_GEAR_RATIO));
        }
    }
    void turnClockwise(double power) {
        runConstantSpeed();
        move(-power, power, -power, power);
    }
    void turnClockwise(int targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", String.format("%.0f", angles.firstAngle));
        double k = 0.005; //experimentally found
        double e = targetAngle + angles.firstAngle; //clockwise is negative for thirdAngle
        double power = (0.05 * e / Math.abs(e)) + k * e;
        power = Range.clip(power, -1.0, 1.0);
        if (Math.abs(e) >= 5)
            turnClockwise(power);
        else {
            stopDriveMotors();
            angleTargetReached = true;
        }
    }

    void turnClockwisePID(int targetAngle) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", String.format("%.0f", angles.firstAngle));
        if (driveVoltage.getVoltage() < 14.0) {
            double kp = 0.010; //proportionality constant (amount to adjust for immediate deviance) experimentally found
            double ki = 0.001; //integral constant (amount to adjust for past errors) experimentally found
            double kd = 0.002; //derivative constant (amount to adjust for future errors) experimentally found
            double e = targetAngle + angles.firstAngle; //error
            e_list.add(e);
            t_list.add(this.time);
            double power = kp*e + ki*integrate() + kd*differentiate();
            power = Range.clip(power, -DRIVE_PWR_MAX, DRIVE_PWR_MAX); //ensure power doesn't exceed max speed
            if (Math.abs(e) >= 5) //5 degree angle slack / uncertainty
                turnClockwise(power);
            else {
                stopDriveMotors();
                e_list.clear();
                t_list.clear();
                angleTargetReached = true;
            }
        }
        else {
            double k = 3.5; //experimentally found
            double power = k * (targetAngle + angles.firstAngle)
                    / Math.abs(targetAngle);
            if (Math.abs(targetAngle + angles.firstAngle) >= 10)
                turnClockwise(power);
            else {
                stopDriveMotors();
                angleTargetReached = true;
            }
        }
    }
    ArrayList<Double> e_list = new ArrayList<>(); //records past errors
    ArrayList<Double> t_list = new ArrayList<>(); // records times past errors took place
    //integrates error of angle w/ respect to time
    double integrate() {
        double sum = 0; //uses trapezoidal sum approximation method
        if (e_list.size() >= 2) {
            for (int i = 0; i <= e_list.size() - 2; i++) {
                double dt = t_list.get(i+1) - t_list.get(i);
                sum += (e_list.get(i+1) + e_list.get(i))*dt / 2.0;
            }
        }
        return sum;
    }
    //differentiates error of angle w/ respect to time
    double differentiate() {
        double slope = 0; //uses secant line approximation
        if (e_list.size() >= 2) {
            double de = e_list.get(e_list.size() - 1) - e_list.get(e_list.size() - 2);
            double dt = t_list.get(t_list.size() - 1) - t_list.get(t_list.size() - 2);
            slope = de/dt;
        }
        return slope;
    }

    /*boolean turnAbsolute(double target) { //Tells robot to rotate to an absolute heading (degrees)
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        boolean absoluteReached = false;
        if (Math.abs(target + angles.firstAngle) <= 10) {
            stopDriveMotors();
            absoluteReached = true;
        }
        return absoluteReached;
    }*/

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
                keyDetected = true;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT)
            { // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                pictographKey = 2;
                keyDetected = true;
            } else if (vuMark == RelicRecoveryVuMark.CENTER)
            { // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                pictographKey = 1;
                keyDetected = true;
            }
        } else
        {
            telemetry.addData("VuMark", "not visible");
            keyDetected = false;
        }
    }
    boolean keyDetected = false;

    void calibrateVariables() {//Used if any autonomous methods need initial state variables
        encoderTargetReached = false;
        angleTargetReached = false;
        colorSensor.enableLed(false);
        jewelKnocked = false;
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }
    //used to measure the amount of time passed since jewel was hit in autonomous has started
    boolean waitJewelSec(double elapsedTime) { return (this.time - jewelTime >= elapsedTime); }

}

