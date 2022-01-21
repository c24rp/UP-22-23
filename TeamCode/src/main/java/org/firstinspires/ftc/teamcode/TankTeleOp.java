package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TankTeleOp", group = "TeleOp")
public class TankTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot rb = new MecanumRobot();

    double liftmotorStartingPosition;
    double servoBoxStartingPosition = 0;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);


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

        intake();


        telemetry.update();
    }



    //moves the lift up
    private void intake() {
        if(gamepad1.right_bumper){
            rb.liftmotor.setTargetPosition(1);
        }
        if(gamepad1.left_bumper){
            rb.liftmotor.setPower(-1);
        }
        else {
            rb.liftmotor.setPower(0);
        }
    }






    private void driveChassis() {
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x * 1;



        if(Math.abs(y) > Math.abs(rx)){
            frontLeftPower = y;
            backLeftPower = y;
            frontRightPower = y;
            backRightPower = y;
        }
        else if(Math.abs(y) < Math.abs(rx)){
            frontLeftPower = rx;
            backLeftPower = rx;
            frontRightPower = -rx;
            backRightPower = -rx;
        }

        else {
            frontLeftPower = 0;
            backLeftPower = 0;
            frontRightPower = 0;
            backRightPower = 0;
        }

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

