package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedAtomicNoDuck")
public class RedAtomicNoDuck extends LinearOpMode {

    /**
     * Amount of time elapsed
     */
    private ElapsedTime runtime = new ElapsedTime();

    MecanumRobot rb = new MecanumRobot();

    public static void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, this);

        rb.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoder(rb.flMotor);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        Thread.sleep(1000);
        waitForStart();

        runtime.reset();

       rb.strafeRightByEncoder(6, rb.flMotor, .4);
        rb.driveForwardByEncoder(27, rb.flMotor, 0.7);
        rb.driveForwardByEncoder(10, rb.flMotor, 0.4);
        rb.driveForwardByEncoder(5, rb.flMotor, 0.3);
        //rb.driveForwardByEncoder(2, rb.flMotor, 0.4);
//        //CHANGE VALUE TO SAME
        //rb.driveForwardByEncoder(-13, rb.flMotor,0.4);
        rb.turnClockwiseByEncoder(-20.5,rb.flMotor,0.3);
     Thread.sleep(1000);
     // Drop off Block
     rb.driveForwardByEncoder(7,rb.flMotor,1.0);
     Thread.sleep(200);
     rb.driveForwardByEncoder(1, rb.flMotor, 0.3);
     // Wait for Atomic
     Thread.sleep(17000);
     // Move backwards away from Caroseul
     rb.driveForwardByEncoder(-5,rb.flMotor,0.5);
     Thread.sleep(200);
     // Turns to strafe into warehouse
     rb.turnClockwiseByEncoder(18, rb.flMotor, .8);
     Thread.sleep(300);
     // Move backwards
     rb.driveForwardByEncoder(-20, rb.flMotor, .7);
     Thread.sleep(200);
     // Turn
        rb.turnClockwiseByEncoder(17,rb.flMotor,0.7);
        Thread.sleep(200);
        // Strafe in front of warehouse
        rb.strafeRightByEncoder(20.75, rb.flMotor, .5);
        Thread.sleep(200);
        // Moves into warehouse
        rb.driveForwardByEncoder(40,rb.flMotor,.7);
      // rb.strafeRightByEncoder(,rb.flMotor,0.9);
      //Thread.sleep(1000);
    // rb.driveForwardByEncoder(60,rb.flMotor,1.0);
    }
}
