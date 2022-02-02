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
        double servoBoxStartingPosition;

        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;

        @Override
        public void init() {
            telemetry.addData("Status", "Initializing");

            rb.init(hardwareMap, null);

            double liftmotorStartingPosition = rb.liftmotor.getCurrentPosition();
            double servoBoxStartingPosition = rb.boxServo.getPosition();


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

            lift();

            intake();

//            liftEncoder();

            servoBox();

            telemetry.update();
        }



        //moves the lift up
        private void lift() {
            if(gamepad1.right_bumper){
                rb.liftmotor.setPower(1);
            }
            else if(gamepad1.left_bumper){
                rb.liftmotor.setPower(-1);
            }
            else {
                rb.liftmotor.setPower(0);
            }
        }

//        private void liftEncoder() {
//            double x = rb.liftmotor.getCurrentPosition();
//            if(gamepad1.dpad_right && x < liftmotorStartingPosition + 400){
//                rb.liftmotor.setPower(1);
//            }
//            if(gamepad1.dpad_left && x > liftmotorStartingPosition){
//                rb.liftmotor.setPower(-1);
//            }
//            else {
//                rb.liftmotor.setPower(0);
//            }
//        }

        private void servoBox() {
            if(gamepad2.dpad_down) {
                rb.boxServo.setPosition(servoBoxStartingPosition - 0.6);
            }
            else if(gamepad2.dpad_right) {
                rb.boxServo.setPosition(servoBoxStartingPosition - 0.25);
            }
            else {
                rb.boxServo.setPosition(servoBoxStartingPosition);
            }

        }


        private void intake() {
            if(gamepad1.x){
                rb.intakemotor.setPower(1);
            }
            else if(gamepad1.b){
                rb.intakemotor.setPower(-1);
            }
            else {
                rb.intakemotor.setPower(0);
            }
        }





        private void driveChassis() {
            double y = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_y * 1;



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
        }
    }


