package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Updated by Alex on 11/5/2017.
 */
@Autonomous(name = "AutoTemplate", group = "default")
public class TestAuto extends GeorgeOp {
    //Declare and Initialize any variables needed for this specific autonomous program

    public TestAuto() {}

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateName, state);
        telemetry.addData("current time", String.format("%.1f", this.time));
        telemetry.addData("state time", String.format("%.1f", this.time - setTime));

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateName = "Initial Calibration";
                resetEncoders();
                state++;
                break;

            case 2:
                stateName = "Drive Forwards";

                moveForward(0.20, 1.60);

                if (encoderTargetReached) {
                    state++;
                }
                break;

            case 4: //Describe the Robot’s Goals & Actions in this state
                stateName = "Drive Backwards";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)

                telemetry.addData("FR", String.format("%.2f", driveFR.getPower()));
                telemetry.addData("FL", String.format("%.2f", driveFL.getPower()));
                telemetry.addData("BR", String.format("%.2f", driveBR.getPower()));
                telemetry.addData("BL", String.format("%.2f", driveBL.getPower()));

                moveForward(-0.20, -1.60);

                if (encoderTargetReached) {
                    state++;
                }
                break;

            case 1000: //Run When Autonomous is Complete
                stateName = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateName = "Calibrating";
                resetEncoders();
                encoderTargetReached = false;
                if (waitSec(1)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program
}