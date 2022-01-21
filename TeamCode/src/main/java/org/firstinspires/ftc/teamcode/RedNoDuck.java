package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedNoDuck")
public class RedNoDuck extends LinearOpMode {

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
// Strafe to the right
       rb.strafeRightByEncoder(6, rb.flMotor, .4);
       // Move forward, slowing down so that the block doesnt fall
        rb.driveForwardByEncoder(27, rb.flMotor, 0.7);
        rb.driveForwardByEncoder(10, rb.flMotor, 0.4);
        rb.driveForwardByEncoder(5, rb.flMotor, 0.3);
        // Turn into the hub
        rb.turnClockwiseByEncoder(-20.5,rb.flMotor,0.3);
     Thread.sleep(1000);
     // Placement, jolts forward
     rb.driveForwardByEncoder(7,rb.flMotor,1.0);
     Thread.sleep(200);
     rb.driveForwardByEncoder(1, rb.flMotor, 0.3);
     Thread.sleep(200);
     // move back
     rb.driveForwardByEncoder(-5,rb.flMotor,0.5);
     Thread.sleep(200);
     // Turn away
     rb.turnClockwiseByEncoder(18, rb.flMotor, .8);
     Thread.sleep(300);
     // Drive backwards
     rb.driveForwardByEncoder(-20, rb.flMotor, .7);
     Thread.sleep(200);
     // turn into the hub (forward looking)
        rb.turnClockwiseByEncoder(17,rb.flMotor,0.7);
        Thread.sleep(200);
        // Strafe in front of the gap
        rb.strafeRightByEncoder(20.75, rb.flMotor, .5);
        Thread.sleep(200);
        // Move into the supply hub (UP THE NUMBER IF NEEDED TO GO FURTHER)
        rb.driveForwardByEncoder(40,rb.flMotor,.7);
    }
}
