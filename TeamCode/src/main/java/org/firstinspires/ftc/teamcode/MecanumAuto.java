package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedDucksideAuto")
public abstract class MecanumAuto extends LinearOpMode {
    private MecanumRobot rb = new MecanumRobot();
    private ElapsedTime time = new ElapsedTime();




    public void RunOpMode(){


        while(opModeIsActive()){
            duck();
            park();
        }

    }

    public void moveToDeposit(){
        // deposit
            // move to location
            // extend arm
            // run the servo for the box
            // reverse servo
            // reverse the arm
            // then run duck code
    }
    public void duck(){
        double currentTime = time.milliseconds();
        while(currentTime < currentTime + 5000){
            rb.turnClockwise(1);
        }
        while(currentTime < currentTime + 8000){
            rb.driveForward(-1);
        }
        while(currentTime < currentTime + 11000){
            rb.moveduck();
        }


        // code to move to the duck from deposit
        // spin the duck
        // run parking code
















    }
    public void park(){
        double currentTime = time.milliseconds();
        while(currentTime < currentTime + 11000){
            rb.driveForward(1);
        }

        // move from duck spin to parking
    }

    public void wait(int waitTime){
       try {
           Thread.sleep(waitTime);
       }
           catch(InterruptedException ex){
               Thread.currentThread().interrupt();
           }
       }

    }

