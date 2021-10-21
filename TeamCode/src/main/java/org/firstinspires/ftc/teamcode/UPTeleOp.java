package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD_SQUARED;
import static org.firstinspires.ftc.teamcode.Constants.LS_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.LS_UP;
import static org.firstinspires.ftc.teamcode.Constants.RS_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.RS_UP;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "UPTeleOp", group = "TeleOp")
@Disabled
    public class UPTeleOp extends OpMode {


    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot rb = new MecanumRobot();


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void start() {
        // Reset elapsed time
        runtime.reset();
    }



    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        driveChassis();

        driveChassis();

        moveDuck();

        IntakeIn();
        IntakeOut();


    }


    private void driveChassis() {
        float leftY = -gamepad1.left_stick_y;
        float leftX = gamepad1.left_stick_x;
        float rightX = gamepad1.right_stick_x;

        double pow;
        if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
            pow = DRIVE_POWER_SLOW;
        } else {
            pow = DRIVE_POWER;
        }

        if (leftX * leftX + leftY * leftY >= DRIVE_STICK_THRESHOLD_SQUARED || Math.abs(rightX) >= DRIVE_STICK_THRESHOLD) {
            rb.drive(leftX, leftY, rightX, pow);
        } else {
            rb.driveStop();
        }
    }

    private void moveDuck() {
        if(gamepad2.a){
            rb.duckmotor.setPower(-0.5);
        }
        else {
            rb.duckmotor.setPower(0);
        }
    }

    private void IntakeIn() {
        if(gamepad2.right_bumper){
            rb.intakemotor.setPower(0.7);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }

    private void IntakeOut() {
        if(gamepad2.left_bumper){
            rb.intakemotor.setPower(-0.7);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }
}