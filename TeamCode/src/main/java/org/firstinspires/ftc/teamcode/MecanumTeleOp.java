package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD_SQUARED;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

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
        runtime.reset();

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        driveChassis();

        moveDuck();
        lift();
        Intake();

//        telemetry.addData("frEncoder", rb.frMotor.getCurrentPosition());
//        telemetry.addData("flEncoder", rb.flMotor.getCurrentPosition());
//        telemetry.addData("blEncoder", rb.blMotor.getCurrentPosition());
//        telemetry.update();

    }

    //moves the the spinning
    private void moveDuck() {
        if(gamepad1.a){
            rb.duckmotor.setPower(-0.5);
        }
        else {
            rb.duckmotor.setPower(0);
        }
    }

    //Intake
    private void Intake() {
        if(gamepad1.right_bumper){
            rb.intakemotor.setPower(1.0);
        }
        if(gamepad1.left_bumper){
            rb.intakemotor.setPower(-1.0);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }

    //moves the lift up
    private void lift() {
        if(gamepad1.b){
            rb.liftmotor.setPower(1);
        }
        if(gamepad1.x){
            rb.liftmotor.setPower(-1);
        }
        else {
            rb.liftmotor.setPower(0);
        }
    }




    private void driveChassis() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x * 0.5;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x - rx) / denominator;
        double frontRightPower = (y - x + rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        rb.flMotor.setPower(frontLeftPower);
        rb.blMotor.setPower(backLeftPower);
        rb.frMotor.setPower(frontRightPower);
        rb.brMotor.setPower(backRightPower);





//        float leftY = -gamepad1.left_stick_y;
//        float leftX = gamepad1.left_stick_x;
//        float rightX = gamepad1.right_stick_x;
//
//        double pow;
//        if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
//            pow = DRIVE_POWER_SLOW;
//        } else {
//            pow = DRIVE_POWER;
//        }
//
//        if (leftX * leftX + leftY * leftY >= DRIVE_STICK_THRESHOLD_SQUARED || Math.abs(rightX) >= DRIVE_STICK_THRESHOLD) {
//            rb.drive(leftX, leftY, rightX, pow);
//        } else {
//            rb.driveStop();
//        }
    }
}

