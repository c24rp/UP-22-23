package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueFoundationAutoEnc", group = "FoundationAuto")
public class Auto extends LinearOpMode {

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

        rb.strafeRightByEncoder(374, rb.flMotor, .4);
        rb.driveForwardByEncoder(-1330, rb.flMotor, .4);
        Thread.sleep(1250);
        rb.driveForwardByEncoder(1280, rb.flMotor, .4);
        rb.turnClockwiseByEncoder(-520,rb.flMotor, .4);
        rb.driveForwardByEncoder(600, rb.flMotor, .4);
        rb.strafeRightByEncoder(-1500, rb.flMotor, .4);
    }
}
