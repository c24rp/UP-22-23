package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_POWER_SLOW;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD;
import static org.firstinspires.ftc.teamcode.Constants.DRIVE_STICK_THRESHOLD_SQUARED;
import static org.firstinspires.ftc.teamcode.Constants.LS_DOWN;
import static org.firstinspires.ftc.teamcode.Constants.LS_UP;
import static org.firstinspires.ftc.teamcode.Constants.TRIGGER_THRESHOLD;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "UPTeleOp", group = "TeleOp")
@Disabled
    public class UPTeleOp extends OpMode {


    private ElapsedTime runtime = new ElapsedTime();

    private MecanumRobot rb = new MecanumRobot();

    double liftmotorStartingPosition;
    double servoBoxStartingPosition;


    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        rb.init(hardwareMap, null);

        telemetry.addData("Status", "Initialized");

        liftmotorStartingPosition = rb.liftmotor.getCurrentPosition();
        liftmotorStartingPosition = rb.boxServo.getPosition();

        rb.resetEncoder(rb.liftmotor);
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
        if(gamepad2.y){
            rb.duckmotor.setPower(-0.5);
        }
        if(gamepad2.a){
            rb.duckmotor.setPower(0.5);
        }
        else {
            rb.duckmotor.setPower(0);
        }
    }

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
        if(gamepad1.dpad_left){
            rb.boxServo.setPosition(-0.5);
        }


        if(gamepad1.dpad_right){
            rb.boxServo.setPosition(0.5);
        }


        else{
            rb.boxServo.setPosition(0);
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

    }
}