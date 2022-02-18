package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "blueducktank")
public class blueducktank extends LinearOpMode {

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
       rb.liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       rb.liftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rb.liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       telemetry.addData("Status", "Initialized");

       telemetry.update();

       waitForStart();
       //lift encoder numbers
       //bottom is 1200
       // middle is 3200

       runtime.reset();
//       encoder auto
       rb.driveForwardByEncoder(-30, rb.blMotor, 1);
       Thread.sleep(500);
       rb.turnClockwiseByEncoder(-7, rb.blMotor, 1);
       Thread.sleep(500);

       rb.liftmotor.setTargetPosition(3200);
       rb.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rb.liftmotor.setPower(0.5);
       Thread.sleep(2500);




       rb.boxServo.setPosition(0.67);
       Thread.sleep(1500);
       rb.boxServo.setPosition(0.15);
       Thread.sleep(1000);

       rb.liftmotor.setTargetPosition(0);
       rb.liftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       rb.liftmotor.setPower(0.25);


       Thread.sleep(2000);




       rb.turnClockwiseByEncoder(-5, rb.blMotor, 1);
       Thread.sleep(500);
       rb.driveForwardByEncoder(40, rb.blMotor, 1);
       Thread.sleep(500);
       rb.turnClockwiseByEncoder(7, rb.blMotor, 1);
       Thread.sleep(500);
       rb.driveForwardByEncoder(6.5, rb.blMotor, 0.4);
       duckByTime(2000, -0.5);
       Thread.sleep(2500);


       rb.turnClockwiseByEncoder(-11.5, rb.blMotor, 1);
       Thread.sleep(500);
       rb.driveForwardByEncoder(-155, rb.blMotor, 1);


//
//       turnClockWiseByTime(1000,-1);
//       driveForwardByTime(2000, 1);
   }

    void duckByTime(double milliseconds, double power) {
        double currTime = getRuntime();
        double waitUntil = currTime + (double)(milliseconds/1000);
        while (getRuntime() < waitUntil){
            rb.duckmotor.setPower(power);
        }
        rb.driveStop();
    }

    void liftByEncoder(double encoder, double power) {
        double currposition = rb.liftmotor.getCurrentPosition();
        while (currposition < encoder){
            rb.liftmotor.setPower(power);

            currposition -= 1;

            telemetry.addData("pos", currposition);
            telemetry.update();

        }
        rb.driveStop();
    }
}

