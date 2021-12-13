package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueSide")
public class BlueSide extends LinearOpMode {

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
        rb.driveForwardByEncoder(19, rb.flMotor, 1);
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
        rb.turnClockwiseByEncoder(-2,rb.flMotor,0.3);
        Thread.sleep(200);

        // Move into warehouse
        rb.driveForwardByEncoder(42,rb.flMotor,0.7);

    }
}
