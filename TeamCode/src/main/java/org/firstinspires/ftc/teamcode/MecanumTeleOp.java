package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MecanumTeleOp", group = "TeleOp")
public class MecanumTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot rb = new MecanumRobot();

    double liftmotorStartingPosition;
    double servoBoxStartingPosition = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);

        liftmotorStartingPosition = rb.liftmotor.getCurrentPosition();
//        servoBoxStartingPosition = rb.boxServo.getPosition();

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        runtime.reset();
        rb.resetEncoder(rb.liftmotor);
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Looping");

        driveChassis();
        moveDuck();
        lift();
        intake();
        servoBox();
    }

    //moves the the spinning wheel
    private void moveDuck() {
        if(gamepad1.a){

            rb.duckmotor.setPower(-0.8);
        }
        if(gamepad1.y){
            rb.duckmotor.setPower(0.8);
        }
        else {
            rb.duckmotor.setPower(0);
        }
    }

    //Intake
    private void lift() {
        if(gamepad2.b){
            rb.intakemotor.setPower(1);
        }
        if(gamepad2.x){
            rb.intakemotor.setPower(-1);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }


    //moves the lift up
    private void intake() {
        if(gamepad2.right_bumper){
            rb.liftmotor.setTargetPosition(1);
        }
        if(gamepad2.left_bumper){
            rb.liftmotor.setPower(-1);
        }
        else {
            rb.liftmotor.setPower(0);
        }
    }

    // this controls the servo that is on the box
    private void servoBox() {
        if(gamepad2.dpad_left){
            rb.boxServo.setPosition(servoBoxStartingPosition + 0.2);

        }
        if(gamepad2.dpad_right){
            rb.boxServo.setPosition(servoBoxStartingPosition - 0.15);
        }

        if(gamepad2.dpad_down){
            rb.boxServo.setPosition(servoBoxStartingPosition);
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


//        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x;
//        final double v1 = r * Math.cos(robotAngle) + rightX;
//        final double v2 = r * Math.sin(robotAngle) - rightX;
//        final double v3 = r * Math.sin(robotAngle) + rightX;
//        final double v4 = r * Math.cos(robotAngle) - rightX;
//
//        rb.flMotor.setPower(v1);
//        rb.blMotor.setPower(v2);
//        rb.frMotor.setPower(v3);
//        rb.brMotor.setPower(v4);


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

