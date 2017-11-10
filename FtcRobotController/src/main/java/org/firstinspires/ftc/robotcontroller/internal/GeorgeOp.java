package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Updated by Alex, Kalvin on 11/10/17
 * GeorgeOp is the TeleOp for the Competition Robot
 * Has Following Robot Systems:
 * --Mecanum Drive Train
 * ----Follows Same Code from MecanumDriveOp
 * ----Adds Gyro Sensor for Autonomous
 * ----Adds turnAbsolute() and turnRelative() methods for autonomous
 * ----Adds calibrateVariables() method for autonomous
 * --Jewel Mechanism
 * ----Adapted from PrototypeJewelOpMode3
 * ----Modifies updateActuators() method [Renamed to updateJewel()] to accommodate Controls for other Systems
 * ----Adds in the Color Sensor for Autonomous
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
    ColorSensor colorSensor; //For Jewel Mechanism
    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyroMR; //For Mecanum Drive Train
    DistanceSensor rangeSensor;
    ModernRoboticsI2cRangeSensor range; //for detecting the wall in autonomous

    //Mecanum Drive Train Variables and Constants
    final double DRIVE_PWR_MAX = 0.90;
    final int COUNTS_PER_REVOLUTION = 1440; //AndyMark Motors
    final double DRIVE_GEAR_RATIO = 16.0 / 24.0; //Driven / Driver
    final double COUNTS_PER_INCH_RF = COUNTS_PER_REVOLUTION / (4 * Math.PI / DRIVE_GEAR_RATIO); //forward / right / backward / left
    final double COUNTS_PER_INCH_DG = COUNTS_PER_REVOLUTION / (2 * Math.PI * Math.sqrt(2) / DRIVE_GEAR_RATIO); //diagonal
    final double WHITE_THRESHOLD = 0.30;
    double forwardRightPower = 0;
    double forwardLeftPower = 0;
    double backwardRightPower = 0;
    double backwardLeftPower = 0;

    //Jewel Mechanism Variables and Constants
    final float UPDOWN_MIN = 0 / 255.0f;
    final float UPDOWN_MAX = 255 / 255.0f;
    final float LEFTRIGHT_MIN = 0 / 255.0f;
    final float LEFTRIGHT_MAX = 255 / 255.0f;
    final int BLUE_THRESHOLD = 2;
    final int RED_THRESHOLD = 2;
    double upDownPos = 0;
    double leftRightPos = 0;
    double jewelDelta = 0.005;

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

        //Initialize servos
        upDownServo = hardwareMap.servo.get("uds");
        leftRightServo = hardwareMap.servo.get("lrs");

        //Initialize sensors
        colorSensor = hardwareMap.colorSensor.get("cs");
        gyroSensor = hardwareMap.gyroSensor.get("gs");
        gyroMR = (ModernRoboticsI2cGyro) gyroSensor;
        range = (ModernRoboticsI2cRangeSensor) rangeSensor;

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
        updateJewel();
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
        //Clip and Initialize Jewel Mechanism
        upDownPos = Range.clip(upDownPos, UPDOWN_MIN, UPDOWN_MAX);
        upDownServo.setPosition(upDownPos);
        leftRightPos = Range.clip(leftRightPos, LEFTRIGHT_MIN, LEFTRIGHT_MAX);
        leftRightServo.setPosition(leftRightPos);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("FR", String.format("%.2f",forwardRightPower));
        telemetry.addData("FL", String.format("%.2f",forwardLeftPower));
        telemetry.addData("BR", String.format("%.2f",backwardRightPower));
        telemetry.addData("BL", String.format("%.2f",forwardLeftPower));
        telemetry.addData("UD", String.format("%.0f", upDownPos * 255));
        telemetry.addData("LR", String.format("%.0f", leftRightPos * 255));
        telemetry.addData("Gyro", gyroMR.getIntegratedZValue());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
    }

    //Create Methods that will update the driver data
    void updateDriveTrain() {
        forwardRightPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        forwardLeftPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        backwardRightPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * DRIVE_PWR_MAX;
        backwardLeftPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * DRIVE_PWR_MAX;
    }
    void updateJewel() {
        if (gamepad2.dpad_up)
            upDownPos += jewelDelta;
        else if (gamepad2.dpad_down)
            upDownPos -= jewelDelta;
        if (gamepad2.dpad_right)
            leftRightPos += jewelDelta;
        else if (gamepad2.dpad_left)
            leftRightPos -= jewelDelta;
    }


    //Create variables/methods that will be used in ALL autonomous programs for this specific robot

    double setTime; //used to measure the time period of each step in autonomous
    int state = 0; //used to control the steps taken during autonomous
    String stateName = ""; //Overwrite this as the specific step used in Autonomous

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
    void stopDriveMotors() {
        driveFR.setPower(0.0);
        driveFL.setPower(0.0);
        driveBR.setPower(0.0);
        driveBL.setPower(0.0);
    }
    void moveForward(double power) {
        driveFR.setPower(power);
        driveFL.setPower(power);
        driveBR.setPower(power);
        driveBL.setPower(power);
    }
    void moveForward(double power, int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_RF);
        driveFR.setTargetPosition(target);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFL.setTargetPosition(target);
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setTargetPosition(target);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setTargetPosition(target);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void moveRight(double power) {
        driveFR.setPower(-power);
        driveFL.setPower(power);
        driveBR.setPower(power);
        driveBL.setPower(-power);
    }
    void moveRight(int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_RF);
        driveFR.setTargetPosition(-target);
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFL.setTargetPosition(target);
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setTargetPosition(target);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setTargetPosition(-target);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void moveForwardRight(double power) {
        driveFR.setPower(0.0);
        driveFL.setPower(power);
        driveBR.setPower(power);
        driveBL.setPower(0.0);
    }
    void moveForwardRight(int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_DG);
        driveFR.setTargetPosition(0);       //motor will not rotate because the motor position resets to 0 at end of each stage
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFL.setTargetPosition(target);
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setTargetPosition(target);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setTargetPosition(0);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void moveForwardLeft(double power) {
        driveFR.setPower(power);
        driveFL.setPower(0.0);
        driveBR.setPower(0.0);;
        driveBL.setPower(power);
    }
    void moveForwardLeft(int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_DG);
        driveFR.setTargetPosition(target);       //motor will not rotate because the motor position resets to 0 at end of each stage
        driveFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveFL.setTargetPosition(0);
        driveFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBR.setTargetPosition(0);
        driveBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driveBL.setTargetPosition(target);
        driveBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    void turnClockwise(double power) {
        driveFR.setPower(-power);
        driveFL.setPower(power);
        driveBR.setPower(-power);
        driveBL.setPower(power);
    }

    boolean turnAbsolute(double target) { //Tells robot to rotate to an absolute heading (degrees)
        boolean absoluteReached = false;
        if (Math.abs(gyroMR.getIntegratedZValue() + target) <= Math.abs(driveFR.getPower() * 95)) {
            stopDriveMotors();
            absoluteReached = true;
        }
        return absoluteReached;
    }
    boolean turnRelative(int target) { //Tells robot to rotate target degrees from starting position
        boolean relativeReached = false;
        target -= startingIntZVal;
        startingIntZVal = 0;
        if (Math.abs(gyroMR.getIntegratedZValue() + target) <= Math.abs(driveFR.getPower() * 95)) {
            stopDriveMotors();
            relativeReached = true;
        }
        return relativeReached;
    }
    int startingIntZVal = 0; //Reset this value to last IntegratedZValue during each Calibrating State


    void calibrateVariables() {//Used if any autonomous methods need initial state variables
        startingIntZVal = gyroMR.getIntegratedZValue();
        colorSensor.enableLed(false);
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

