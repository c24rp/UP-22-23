package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueNoDuck")
public class BlueNoDuck extends LinearOpMode {

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

        resetEncoder(rb.frMotor);
        resetEncoder(rb.flMotor);
        resetEncoder(rb.blMotor);
        resetEncoder(rb.brMotor);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        Thread.sleep(1000);
        waitForStart();

        runtime.reset();

       rb.strafeRightByEncoder(-6, rb.flMotor, .4);
        rb.driveForwardByEncoder(26, rb.flMotor, 0.7);
        rb.driveForwardByEncoder(10, rb.flMotor, 0.4);
        rb.driveForwardByEncoder(5, rb.flMotor, 0.3);
        //rb.driveForwardByEncoder(2, rb.flMotor, 0.4);
//        //CHANGE VALUE TO SAME
        //rb.driveForwardByEncoder(-13, rb.flMotor,0.4);
        rb.turnClockwiseByEncoder(18,rb.flMotor,0.3);
     Thread.sleep(1000);
     rb.driveForwardByEncoder(5,rb.flMotor,1.0);
     Thread.sleep(200);
     rb.driveForwardByEncoder(2, rb.flMotor, 0.3);
     Thread.sleep(200);
     rb.turnClockwiseByEncoder(-18, rb.flMotor, .8);
     Thread.sleep(300);
     rb.driveForwardByEncoder(-20, rb.flMotor, .7);
     Thread.sleep(200);
        rb.turnClockwiseByEncoder(-17.5,rb.flMotor,0.7);
        Thread.sleep(200);
        rb.strafeRightByEncoder(-21, rb.flMotor, .5);
        Thread.sleep(200);
        rb.driveForwardByEncoder(30,rb.flMotor,.7);
      // rb.strafeRightByEncoder(,rb.flMotor,0.9);
      //Thread.sleep(1000);
    // rb.driveForwardByEncoder(60,rb.flMotor,1.0);
    }
}
