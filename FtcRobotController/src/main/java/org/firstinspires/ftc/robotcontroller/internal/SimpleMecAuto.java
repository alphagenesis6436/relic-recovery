package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Alex on 11/1/2017.
 */
@Autonomous (name = "SimpleMecAuto", group = "default")
@Disabled
public class SimpleMecAuto extends MecanumDriveOp {
    //Declare and Initialize any variables needed for this specific autonomous program

    public SimpleMecAuto() {}

    @Override
    public void loop(){
        //Display Data to be displayed throughout entire Autonomous
        telemetry.addData(stateName, state);
        telemetry.addData("current time", this.time);
        telemetry.addData("state time", this.time - setTime);

        //Use Switch statement to proceed through Autonomous strategy (only use even cases for steps)
        switch(state){
            case 0: //Use this state to reset all hardware devices
                stateName = "Initial Calibration";
                resetSensors();
                //resetEncoders();
                state++;
                break;

            case 2: //Describe the Robot’s Goals & Actions in this state
                stateName = "Drive Forward 1 second";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                moveForward(0.40);

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    stopDriveMotors();
                    state++;
                }
                break;

            case 4: //Describe the Robot’s Goals & Actions in this state
                stateName = "Stop for 1 second";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                stopDriveMotors();

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    state++;
                }
                break;


            case 6: //Describe the Robot’s Goals & Actions in this state
                stateName = "Drive Right 1 seconds";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                moveRight(0.40);

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    stopDriveMotors();
                    state++;
                }
                break;

            case 8: //Describe the Robot’s Goals & Actions in this state
                stateName = "Stop for 1 second";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                stopDriveMotors();

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    state++;
                }
                break;


            case 10: //Describe the Robot’s Goals & Actions in this state
                stateName = "Drive Backward Left 1 seconds";
                //Display any current data needed to be seen during this state (if none is needed, omit this comment)
                moveForwardRight(-0.4);

                if (waitSec(1)) { //Use a boolean value that reads true when state goal is completed
                    stopDriveMotors();
                    state++;
                }
                break;

            case 1000: //Run When Autonomous is Complete
                stateName = "Autonomous Complete";
                //Set all motors to zero and servos to initial positions
                resetSensors();
                //resetEncoders();
                break;

            default://Default state used to reset all hardware devices to ensure no errors
                stateName = "Calibrating";
                resetSensors();
                //resetEncoders();
                if (waitSec(1)) {
                    state++;
                    setTime = this.time;
                }
                break;
        }
    }

    //Create any methods needed for this specific autonomous program

}

