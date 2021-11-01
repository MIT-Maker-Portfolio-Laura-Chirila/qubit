package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.json.JSONException;

import java.io.IOException;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class Autonomous extends LinearOpMode {

    private Robot robot;

    private void fixErrors(){
        msStuckDetectStop = 20000;
        msStuckDetectStart = 20000;
        msStuckDetectInit = 20000;
        msStuckDetectInitLoop = 20000;
        msStuckDetectLoop = 20000;
    }

    private boolean autonomousFinished = false;
    public void initialize() {

        this.robot = new Robot(this.hardwareMap, this.telemetry);
        this.robot.initAutonomous();
        this.fixErrors();
    }

    @Override
    public void runOpMode() {
        this.initialize();

        //waitForStart();

        while(isStarted() == false && isStopRequested() == false){
            this.telemetry.addData("Status: ", "waiting for start command... time = %.02f",getRuntime());
            this.telemetry.update();
            sleep(200);

        }

        //This sets the timer to 0 when the opmode starts
        //runtime.reset();
        //While the opmode is active, this block of code is run. Once autonomous is finished, the boolean will be true
        while(opModeIsActive()){
//            if(autonomousFinished){
//                robot.stopAllMovements();
//            } else {
                try {
                    this.robot.autonomous();
                } catch (IOException e) {
                    e.printStackTrace();
                } catch (JSONException e) {
                    e.printStackTrace();
                }
//                autonomousFinished = true;
//            }
        }
    }
}
