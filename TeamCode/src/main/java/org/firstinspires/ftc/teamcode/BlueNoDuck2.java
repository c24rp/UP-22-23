package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueNoDuck2")
public class BlueNoDuck2 extends LinearOpMode {

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
        // strafe to the left
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

        rb.strafeRightByEncoder(-6, rb.frMotor, .4);
        rb.driveForwardByEncoder(-27, rb.frMotor, 0.7);
        rb.driveForwardByEncoder(-10, rb.frMotor, 0.4);
        rb.driveForwardByEncoder(-5, rb.frMotor, 0.3);
        //rb.driveForwardByEncoder(2, rb.flMotor, 0.4);
//        //CHANGE VALUE TO SAME
        //rb.driveForwardByEncoder(-13, rb.flMotor,0.4);
        rb.turnClockwiseByEncoder(20.5,rb.frMotor,0.3);
        Thread.sleep(1000);
        // Drop off Block
        rb.driveForwardByEncoder(-7,rb.frMotor,1.0);
        Thread.sleep(200);
        rb.driveForwardByEncoder(-1, rb.frMotor, 0.3);
        // Wait for Atomic
        Thread.sleep(17000);
        // Move backwards away from Carosel
        rb.driveForwardByEncoder(5,rb.frMotor,0.5);
        Thread.sleep(200);
        // Turns to strafe into warehouse
        rb.turnClockwiseByEncoder(-18, rb.frMotor, .8);
        Thread.sleep(300);
        // Move backwards
        rb.driveForwardByEncoder(20, rb.frMotor, .7);
        Thread.sleep(200);
        // Turn
        rb.turnClockwiseByEncoder(-17,rb.frMotor,0.7);
        Thread.sleep(200);
        // Strafe in front of warehouse
        rb.strafeRightByEncoder(-20.75, rb.frMotor, .5);
        Thread.sleep(200);
        // Moves into warehouse
        rb.driveForwardByEncoder(-40,rb.frMotor,.7);
    }
}
