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

        resetEncoder(rb.flMotor);

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        Thread.sleep(1000);
        waitForStart();

        runtime.reset();
        // strafe to the left
       rb.strafeRightByEncoder(-13, rb.flMotor, .4);
        // Move forward
        rb.driveForwardByEncoder(26, rb.flMotor, 0.7);
        rb.driveForwardByEncoder(8, rb.flMotor, 0.1);
        rb.turnClockwiseByEncoder(18,rb.flMotor,0.25);
     Thread.sleep(1000);
     // Jolts forward for placement
     rb.driveForwardByEncoder(4,rb.flMotor,1.0);
     Thread.sleep(200);
        rb.driveForwardByEncoder(2,rb.flMotor,0.3);
        Thread.sleep(200);
        // Move back
        rb.driveForwardByEncoder(-2,rb.flMotor,0.7);
        // Turn away
     rb.turnClockwiseByEncoder(-19, rb.flMotor, .8);
     Thread.sleep(300);
     // Drive backwards
     rb.driveForwardByEncoder(-20, rb.flMotor, .7);
     Thread.sleep(200);
     // Turn toward warehouse
        rb.turnClockwiseByEncoder(-17.5,rb.flMotor,0.7);
        Thread.sleep(200);
        // Strafe in front
        rb.strafeRightByEncoder(-19,rb.flMotor,0.7);
        // Move into parking warehouse
        rb.driveForwardByEncoder(40,rb.flMotor,.7);
    }
}
