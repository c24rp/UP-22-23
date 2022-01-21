package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoTest (DOESN'T WORK)")
public class AutoTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    MecanumRobot rb = new MecanumRobot();
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.5 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    @Override
    public void runOpMode() {
        rb.init(hardwareMap, this);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        rb.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rb.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                rb.flMotor.getCurrentPosition(),
                rb.frMotor.getCurrentPosition(),
                rb.brMotor.getCurrentPosition(),
                rb.blMotor.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  24,  24, 4);  // S1: Forward 47 Inches with 5 Sec timeout
        //encoderDrive(TURN_SPEED,   1, -1, 4);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 3, -3, 4);  // S3: Reverse 24 Inches with 4 Sec timeout

        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = rb.flMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = rb.blMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rb.brMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightFrontTarget = rb.frMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            rb.flMotor.setTargetPosition(newLeftFrontTarget);
            rb.frMotor.setTargetPosition(newLeftBackTarget);
            rb.brMotor.setTargetPosition(newRightBackTarget);
            rb.blMotor.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            rb.flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            rb.flMotor.setPower(Math.abs(speed));
            rb.frMotor.setPower(Math.abs(speed));
            rb.blMotor.setPower(Math.abs(speed));
            rb.brMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (rb.flMotor.isBusy() && rb.frMotor.isBusy() && rb.blMotor.isBusy() && rb.brMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget,newLeftBackTarget, newRightBackTarget, newRightFrontTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        rb.flMotor.getCurrentPosition(),
                        rb.frMotor.getCurrentPosition(),
                        rb.blMotor.getCurrentPosition(),
                        rb.brMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            rb.flMotor.setPower(0);
            rb.frMotor.setPower(0);
            rb.blMotor.setPower(0);
            rb.brMotor.setPower(0);


            // Turn off RUN_TO_POSITION
            rb.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
}
