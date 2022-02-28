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

    double liftmotorStartingPosition = 0;
    double servoPosition = 0;

    double x = 0;
    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);

        liftmotorStartingPosition = rb.liftmotor.getCurrentPosition();

        //rb.resetEncoder(rb.liftmotor);


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

        liftEncoder();

        intake();

//            servoTest();

        servoBox();

        duck();

        telemetry.update();
    }



    private void liftEncoder() {

        double x = rb.liftmotor.getCurrentPosition();
        telemetry.addData("lift_current_pos", x);
        telemetry.update();

        if(gamepad2.dpad_up && rb.liftmotor.getCurrentPosition() < 3770){
            rb.liftmotor.setPower(1);
        }
        else if(gamepad2.dpad_down && rb.liftmotor.getCurrentPosition() > 0 ){
            rb.liftmotor.setPower(-0.5);
        }



        else {
            rb.liftmotor.setPower(0);
        }
    }

    private void lift() {

        double x = rb.liftmotor.getCurrentPosition();
        telemetry.addData("lift_current_pos", x);
        telemetry.update();

        if(gamepad2.dpad_up){
            rb.liftmotor.setPower(0.3);
        }
        else if(gamepad2.dpad_down){
            rb.liftmotor.setPower(-0.5);
        }



        else {
            rb.liftmotor.setPower(0);
        }
    }

    private void servoBoxbad() {
        double x = rb.boxServo.getPosition();
        telemetry.addData("servo_getPos", x);
//            two button method:
        if (gamepad2.a && servoPosition <= 0.9) {
            servoPosition += 0.1;

        }
        if (gamepad2.b && servoPosition >= 0.1) {
            servoPosition -= 0.1;
        }

        rb.boxServo.setPosition(servoPosition);
    }

        private void servoBox(){
            double x = rb.boxServo.getPosition();
            telemetry.addData("servo_getPos", x);
//            two button method:
            if (gamepad2.a){
                rb.boxServo.setPosition(0.92);

            }
            if (gamepad2.b){
                rb.boxServo.setPosition(0.40);
            }

            if (gamepad2.x){
                rb.boxServo.setPosition(0.25);
            }

            if (gamepad2.y){
                rb.boxServo.setPosition(0.6);

            }




//            one button method:
//            if(gamepad2.a){
//                rb.boxServo.setPosition(rb.boxServo.getPosition() + 0.1);
//
//            }
//            else {
//                rb.boxServo.setPosition(servoBoxStartingPosition);
//            }

    }

//        private void servoTest() {
//            if(gamepad2.dpad_up) {
//                telemetry.addData("Servo", rb.boxServo.getPosition());
//                rb.boxServo.setPosition(x);
//                x += 0.04;
//            }
//
//        }


    private void intake() {
        if(gamepad2.right_bumper){
            rb.intakemotor.setPower(1);
        }
        else if(gamepad2.left_bumper){
            rb.intakemotor.setPower(-1);
        }
        else {
            rb.intakemotor.setPower(0);
        }
    }

    private void duck() {
        if(gamepad1.a){
            rb.duckmotor.setPower(0.85); // Test if either 1 or -1
        }
        else if(gamepad1.y){
            rb.duckmotor.setPower(-0.85); // Test if either 1 or -1
        }
        else {
            rb.duckmotor.setPower(0);
        }
    }


    private void driveChassis() {
        telemetry.addData("rightfront_getPos", rb.frMotor.getCurrentPosition());
        telemetry.addData("leftfront_getPos", rb.flMotor.getCurrentPosition());
        telemetry.addData("rightback_getPos", rb.brMotor.getCurrentPosition());
        telemetry.addData("leftback_getPos", rb.blMotor.getCurrentPosition());
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_y;


        rb.flMotor.setPower(-y);
        rb.blMotor.setPower(y);
        rb.frMotor.setPower(rx);
        rb.brMotor.setPower(rx);
    }



    private void driveChassis2() {
        double y = gamepad1.left_stick_y * 1;
        double rx = gamepad1.right_stick_x * 0.8;

        if(Math.abs(y) > Math.abs(rx)){
            frontLeftPower = -y;
            backLeftPower = y;
            frontRightPower = -y;
            backRightPower = -y;
        }
        else if(Math.abs(y) < Math.abs(rx)){
            frontLeftPower = -rx;
            backLeftPower = rx;
            frontRightPower = rx;
            backRightPower = rx;
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
    }
}


