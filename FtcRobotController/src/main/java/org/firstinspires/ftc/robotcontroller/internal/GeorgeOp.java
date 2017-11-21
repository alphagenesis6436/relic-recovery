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
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
 * ----Declared and Initialized stoneServo
 * ----Created Variables to control stoneServo during teleop
 * ----Create method updateStone() to control stone mechanism
 * --Vuforia System
 * ----Follows Same Code from VuforiaTestOp
 * ----Adds _____ method for autonomous
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
    Servo stoneServo;    //Plastic Gear, 180
    ModernRoboticsI2cColorSensor colorSensor; //For Jewel Mechanism
    ModernRoboticsI2cGyro gyroMR; //For Mecanum Drive Train
    ModernRoboticsI2cRangeSensor range; //for detecting the wall in autonomous

    //Mecanum Drive Train Variables and Constants
    final double DRIVE_PWR_MAX = 0.70;
    final double TURN_PWR_MAX = 0.70;
    final int COUNTS_PER_REVOLUTION = 1120; //AndyMark Motors
    final double DRIVE_GEAR_RATIO = 16.0 / 24.0; //Driven / Driver
    final double COUNTS_PER_INCH_RF = COUNTS_PER_REVOLUTION / (4 * Math.PI / DRIVE_GEAR_RATIO); //forward / right / backward / left
    final double COUNTS_PER_INCH_DG = COUNTS_PER_REVOLUTION / (2 * Math.PI * Math.sqrt(2) / DRIVE_GEAR_RATIO); //diagonal
    final double WHITE_THRESHOLD = 0.30;
    double forwardRightPower = 0;
    double forwardLeftPower = 0;
    double backwardRightPower = 0;
    double backwardLeftPower = 0;

    //Jewel Mechanism Variables and Constants
    final float LEFTRIGHT_MID = 110 / 255.0f;
    final float UPDOWN_MIN = 65 / 255.0f;   //fully down
    final float UPDOWN_MAX = 210 / 255.0f;  //fully up
    final float LEFTRIGHT_MIN = 70 / 255.0f; //far right
    final float LEFTRIGHT_MAX = 140 / 255.0f;   //far left
    final int BLUE_THRESHOLD = 3;   //holes
    final int RED_THRESHOLD = 3;    //holes
    double upDownPos = UPDOWN_MAX;
    double leftRightPos = LEFTRIGHT_MID;
    double jewelDelta = 0.005;

    //Stone Mechanism Variables and Constants
    final float STONE_DOWN = 172 / 255.0f;
    final float STONE_UP = 42 / 255.0f;
    double stonePos = STONE_DOWN;
    double stoneDelta = 0.030;

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

        //Initialize servos
        upDownServo = hardwareMap.servo.get("uds");
        leftRightServo = hardwareMap.servo.get("lrs");
        stoneServo = hardwareMap.servo.get("stone");

        //Initialize sensors
        colorSensor = (ModernRoboticsI2cColorSensor) hardwareMap.colorSensor.get("cs");
        gyroMR = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gs");
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "r");
        colorSensor.enableLed(true);

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
        updateJewel();
        updateStone();
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
        //clip and initialize Stone Mechanism
        stonePos = Range.clip(stonePos, STONE_UP, STONE_DOWN);
        stoneServo.setPosition(stonePos);
    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("FR", String.format("%.2f",driveFR.getPower()));
        telemetry.addData("FL", String.format("%.2f",driveFL.getPower()));
        telemetry.addData("BR", String.format("%.2f",driveBR.getPower()));
        telemetry.addData("BL", String.format("%.2f",driveBL.getPower()));
        telemetry.addData("UD", String.format("%.0f", upDownServo.getPosition() * 255));
        telemetry.addData("LR", String.format("%.0f", leftRightServo.getPosition() * 255));
        telemetry.addData("Gyro", gyroMR.getIntegratedZValue());
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Distance", range.getDistance(DistanceUnit.INCH) + " in.");
        telemetry.addData("Stone", String.format("%.0f", stoneServo.getPosition() * 255));
    }

    //Create Methods that will update the driver data
    void updateDriveTrain() {
        forwardRightPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        forwardLeftPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        backwardRightPower = (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
        backwardLeftPower = (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x * TURN_PWR_MAX) * DRIVE_PWR_MAX;
    }
    void updateJewel() {
        if (gamepad2.dpad_up)
            upDownPos += jewelDelta;
        else if (gamepad2.dpad_down)
            upDownPos -= jewelDelta;
        if (gamepad2.dpad_right)
            leftRightPos -= jewelDelta;
        else if (gamepad2.dpad_left)
            leftRightPos += jewelDelta;
    }
    void updateStone() {
        if (gamepad1.dpad_down)
            stonePos -= stoneDelta;
        else if (gamepad1.dpad_up)
            stonePos += stoneDelta;
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
        }
        stopDriveMotors();
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
        }
        stopDriveMotors();
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
        }
        stopDriveMotors();
    }
    void moveForwardLeft(double power) {
        runConstantSpeed();
        move(power, 0.0, 0.0, power);
    }
    void moveForwardLeft(double power, int inches) {
        int target = (int)Math.round(inches * COUNTS_PER_INCH_DG);

        driveFR.setTargetPosition(target);       //motor will not rotate because the motor position resets to 0 at end of each stage
        driveFL.setTargetPosition(0);
        driveBR.setTargetPosition(0);
        driveBL.setTargetPosition(target);
        runToPosition();
        move(power, 0.0, 0.0, power);

        while (driveFR.isBusy()) {
            //Wait until target position is reached
        }
        stopDriveMotors();
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

    void calibrateVariables() {//Used if any autonomous methods need initial state variables
        colorSensor.enableLed(false);
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }

}

