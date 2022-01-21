package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedDuck")
public class RedDuck extends LinearOpMode {

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

// Drop off block
        rb.driveForwardByEncoder(18, rb.flMotor, 1);
        rb.driveForwardByEncoder(3,rb.flMotor,0.3);
        Thread.sleep(200);
        // Move back
        rb.driveForwardByEncoder(-10,rb.flMotor,0.7);
        Thread.sleep(200);
        // Turn to align backwards toward duc
        rb.turnClockwiseByEncoder(18,rb.flMotor,0.7);
        Thread.sleep(200);
        // Strafe to align to duck
        rb.strafeRightByEncoder(13,rb.flMotor,0.7);
        Thread.sleep(200);
        // Move toward duck (IMPORTANT TO SLOW DOWN SO WE DONT BOUNCE OFF)
        rb.driveForwardByEncoder(-30,rb.flMotor,0.7);
        rb.driveForwardByEncoder(-7,rb.flMotor,0.3);
        rb.driveForwardByEncoder(-6,rb.flMotor,0.1);
        // Tries to align with the carosuel to spin the duck
        rb.turnClockwiseByEncoder(-9,rb.flMotor,0.4);
        Thread.sleep(200);
        rb.duckmotor.setPower(-0.5); // Spin duck motor for 3 seconds)
        Thread.sleep(3000);
        rb.duckmotor.setPower(0);
        // Turn back to drive toward warehouse
        rb.turnClockwiseByEncoder(6,rb.flMotor,0.4);
        // Drive into warehouse while strafing
        rb.driveForwardByEncoder(10,rb.flMotor,0.7);
        rb.driveForwardByEncoder(10,rb.flMotor,0.7);
        rb.strafeRightByEncoder(1, rb.flMotor, 0.3);
        rb.driveForwardByEncoder(10,rb.flMotor,0.7);
        rb.driveForwardByEncoder(10,rb.flMotor,0.7);
        rb.strafeRightByEncoder(1, rb.flMotor, 0.3);
        rb.driveForwardByEncoder(10,rb.flMotor,0.7);
        rb.driveForwardByEncoder(10,rb.flMotor,0.7);
        rb.strafeRightByEncoder(1, rb.flMotor, 0.3);
        rb.driveForwardByEncoder(10,rb.flMotor,0.7);
        // Push deeper into warehouse
        rb.driveForwardByEncoder(25,rb.flMotor,0.7);

    }
}
