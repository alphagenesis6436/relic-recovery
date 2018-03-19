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
@TeleOp(name = "BattleRamOp", group = "Default")
public class BattleRamOp extends GeorgeOp {

    public BattleRamOp() {}

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
    }

    @Override public void start() {
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

    }
    void telemetry() {
        //Show Data for Specific Robot Mechanisms
        telemetry.addData("FR Pwr", String.format("%.2f",driveFR.getPower()));
        telemetry.addData("FL Pwr", String.format("%.2f",driveFL.getPower()));
        telemetry.addData("BR Pwr", String.format("%.2f",driveBR.getPower()));
        telemetry.addData("BL Pwr", String.format("%.2f",driveBL.getPower()));
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
            forwardRightPower *= 0.30;
            forwardLeftPower *= 0.30;
            backwardRightPower *= 0.30;
            backwardLeftPower *= 0.30;
        }

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
        if (gamepad2.x && !glpyhBtnPressed) {
            glpyhBtnPressed = true;
            clawIsUpRight = !clawIsUpRight;
        }
        else if (!gamepad2.x && glpyhBtnPressed)
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
        angleTargetReached = false;
        colorSensor.enableLed(false);
        jewelKnocked = false;
    }
    //used to measure the amount of time passed since a new step in autonomous has started
    boolean waitSec(double elapsedTime) { return (this.time - setTime >= elapsedTime); }
    //used to measure the amount of time passed since jewel was hit in autonomous has started
    boolean waitJewelSec(double elapsedTime) { return (this.time - jewelTime >= elapsedTime); }

}

