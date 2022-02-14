package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class MecanumRobot {
    // --------------------------------------
    // Declare motors

    public static final double THETA_TOLERANCE = 0.04;
    public static double XY_TOLERANCE = 0.05;
    DcMotor flMotor, frMotor, blMotor, brMotor, duckmotor, intakemotor, liftmotor;
    Servo boxServo = null;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.937 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    LinearOpMode opMode;

    void init(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        flMotor = hardwareMap.get(DcMotor.class, "frontleft");
        frMotor = hardwareMap.get(DcMotor.class, "frontright");
        blMotor = hardwareMap.get(DcMotor.class, "backleft");
        brMotor = hardwareMap.get(DcMotor.class, "backright");
        duckmotor = hardwareMap.get(DcMotor.class, "duck");
        intakemotor = hardwareMap.get(DcMotor.class, "intake");
        liftmotor = hardwareMap.get(DcMotor.class, "lift");
        boxServo = hardwareMap.get(Servo.class, "boxServo");


        // Set motor directions
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        frMotor.setDirection(DcMotor.Direction.FORWARD);
        blMotor.setDirection(DcMotor.Direction.FORWARD);
        brMotor.setDirection(DcMotor.Direction.FORWARD);
        duckmotor.setDirection(DcMotor.Direction.REVERSE);
        liftmotor.setDirection(DcMotor.Direction.REVERSE);
        intakemotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake when power is zero
        flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duckmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakemotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void resetEncoder(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void drive(double x_stick, double y_stick, double x_right_stick, double multiplier) {
        if (Math.abs(x_stick) >= (2 * Math.abs(y_stick)) + .1) {
            flMotor.setPower(x_stick * multiplier);
            frMotor.setPower(-x_stick * multiplier);
            blMotor.setPower(-x_stick * multiplier);
            brMotor.setPower(x_stick * multiplier);
        } else {
            flMotor.setPower((y_stick + x_right_stick) * multiplier);
            frMotor.setPower((y_stick - x_right_stick) * multiplier);
            blMotor.setPower((y_stick + x_right_stick) * multiplier);
            brMotor.setPower((y_stick - x_right_stick) * multiplier);
        }
    }

    /**
     * Stop the drive motors
     */
    void driveStop() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    void driveForward(double power) {
        flMotor.setPower(-power);
        blMotor.setPower(-power);
        frMotor.setPower(-power);
        brMotor.setPower(-power);
    }

    void strafeRight(double power) {
        flMotor.setPower(power);
        blMotor.setPower(-power);
        frMotor.setPower(-power);
        brMotor.setPower(power);
    }

    void turnClockwise(double power) {
        flMotor.setPower(power);
        blMotor.setPower(power);
        frMotor.setPower(power);
        brMotor.setPower(power);
    }


    void moveduck() {
        duckmotor.setPower(1);
    }

    void intakeIn() { intakemotor.setPower(1.0); }
    void IntakeOut (){
        intakemotor.setPower(-1.0);
    }

    void duckzero () {
        intakemotor.setPower(0);
    }

    void intakezero (){
        duckmotor.setPower(0);
    }

    void liftUp(){
        liftmotor.setPower(1.0);
    }

    public void hammerBack(){
        boxServo.setPosition(0.5);
    }

    public void hammerPush(){ boxServo.setPosition(-0.5); }



    void driveForwardByEncoder(double inches, DcMotor motor, double power) {
        power = Math.abs(power);
        double positionChange = inches * COUNTS_PER_INCH;
        int oldPosition = motor.getCurrentPosition();
        double targetPosition = oldPosition - positionChange; // minus not plus, flipped motor

        if (positionChange > 0) {
            driveForward(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                opMode.telemetry.addData("auto", motor.getCurrentPosition());
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            driveForward(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }

    void strafeRightByEncoder(double inches, DcMotor motor, double power) {
        power = Math.abs(power);
        double positionChange = COUNTS_PER_INCH * inches;
        int oldPosition = motor.getCurrentPosition();
        double targetPosition = oldPosition + positionChange;

        if (positionChange > 0) {
            strafeRight(power);
            while (opMode.opModeIsActive() && -motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            strafeRight(-power);
            while (opMode.opModeIsActive() && -motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        }

    }
    void turnClockwiseByEncoder (double inches, DcMotor motor, double power){
        double positionChange = COUNTS_PER_INCH * inches;
        power = Math.abs(power);
        int oldPosition = motor.getCurrentPosition();
        double targetPosition = oldPosition - positionChange;

        if (positionChange > 0) {
            turnClockwise(power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() > targetPosition) {
                Thread.yield();
            }
            driveStop();
        } else if (positionChange < 0) {
            turnClockwise(-power);
            while (opMode.opModeIsActive() && motor.getCurrentPosition() < targetPosition) {
                Thread.yield();
            }
            driveStop();
        }
    }


}