package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class EncoderArmTest {

    @TeleOp(name = "Lift Arm Test : ")
    public class TeleopTutorial extends LinearOpMode {

        private DcMotor lift;
        private DcMotor armRight;

        @Override
        public void runOpMode() throws InterruptedException {
            lift = hardwareMap.dcMotor.get("lift");

            // Set lift encoder to 0
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set arm run mode
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Zero Power Behavior
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Setup Telemetry, will not clear after cycle, setup reusable items for output
            telemetry.setAutoClear(false);
            telemetry.addData("Left Arm Position", lift.getCurrentPosition());
            telemetry.addData("Right Arm Position", armRight.getCurrentPosition());





            int armTarget = 0;
            double armSpeed = 0;
            String armCurrentDirection = "up";

            while (opModeIsActive()) {

                if (gamepad1.b) { // Arm UP
                    armTarget = 200;
                    armSpeed = 0.98;
                    armCurrentDirection = "up";

                    lift.setPower(armSpeed);
                    lift.setTargetPosition(armTarget);

                } else if (gamepad1.x) { // Arm DOWN
                    armTarget = 0;
                    armSpeed = -0.1;  // From my research, negative is ignore, so I don't understand why this *seemed* to work
                    armCurrentDirection = "down";

                    lift.setPower(armSpeed);
                    lift.setTargetPosition(armTarget);
                }

                // Remove Power from the Arm Motor if motor is close to 0 position, arm should drop
                if (armCurrentDirection ==  "down" && (lift.getTargetPosition() < 5 || armRight.getTargetPosition() < 5)) {
                    armSpeed = 0;
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }



                idle();
            }
        }
    }
}