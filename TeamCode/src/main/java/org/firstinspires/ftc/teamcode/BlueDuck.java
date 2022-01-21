package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueDuck")
public class BlueDuck extends LinearOpMode {

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

// Moves forward to place
        rb.driveForwardByEncoder(18, rb.flMotor, 1);
        rb.driveForwardByEncoder(3,rb.flMotor,0.3);
        Thread.sleep(200);
        // Move backwards
        rb.driveForwardByEncoder(-10,rb.flMotor,0.7);
        Thread.sleep(200);
        // Turn away
        rb.turnClockwiseByEncoder(-18,rb.flMotor,0.7);
        Thread.sleep(200);
        // Strafe toward wall
        rb.strafeRightByEncoder(-13,rb.flMotor,0.7);
        Thread.sleep(200);
        // Drive toward the duck, moving slower so as to not bounce
        rb.driveForwardByEncoder(-30,rb.flMotor,0.7);
        rb.driveForwardByEncoder(-7,rb.flMotor,0.1);
        rb.driveForwardByEncoder(-5,rb.flMotor,0.05);
        // Spin duck for 3 seconds
        Thread.sleep(200);
        rb.duckmotor.setPower(0.5);
        Thread.sleep(3000);
        rb.duckmotor.setPower(0);
        // Move toward warehouse
        rb.driveForwardByEncoder(50,rb.flMotor,0.7);
        // Turn into the wall so we can get through, LOL
        rb.turnClockwiseByEncoder(-2,rb.flMotor,0.3);
        // Move into warehouse
        rb.driveForwardByEncoder(60,rb.flMotor,0.7);

    }
}
