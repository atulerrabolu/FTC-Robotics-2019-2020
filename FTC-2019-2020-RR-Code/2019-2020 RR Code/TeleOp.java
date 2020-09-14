package org.firstinspires.ftc.teamcode;

/* Created Atul Errabolu and Kush on 7/25/2019 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class TeleOp extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, lift;
    private boolean direction, togglePrecision;
    private double factor;
    private Servo dinesh, kush, saharsh, atul;
    // private Servo[] satul = new Servo[2];
    private PID pid = new PID(.025, 0.0003, 0.0023);
    boolean reverse;
    int reverseFactor;
    private BNO055IMU imu;
    private ElapsedTime runtime;
    private double servo;
    @Override
    public void init() {
        //Maps all the variables to its respective hardware
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFrontDrive");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftRearDrive");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFrontDrive");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightRearDrive");
        lift = (DcMotorEx) hardwareMap.dcMotor.get("lift");
        dinesh = hardwareMap.servo.get("servoright");
        kush = hardwareMap.servo.get("servoleft");
        saharsh = hardwareMap.servo.get("lift1");
        atul = hardwareMap.servo.get("lift2");
        // satul = {hardwaremap.servo.get("servoright"), hardwaremap.servo.get("servoleft")};
        //Initialize all the hardware to use Encoders
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize the motors to begin stationary
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Left Motors are in reverse and Right Motors are forward so the robot can move forward
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        saharsh.setDirection(Servo.Direction.FORWARD);
        atul.setDirection(Servo.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        runtime = new ElapsedTime();
    }


    @Override
    public void loop() {
        if(gamepad1.a && !reverse)
            reverse = true;
        else if(reverse && gamepad1.a)
            reverse = false;

        //toggles precision mode if the right stick button is pressed
        if (gamepad2.right_trigger> .5 || gamepad1.right_trigger> .5)
            togglePrecision = true;
        else if (gamepad2.left_trigger<.4999 || gamepad1.left_trigger<.499999)
            togglePrecision = false;

        //sets the factor multiplied to the power of the motors
        factor = togglePrecision ? .3 : 1; //the power is 1/5th of its normal value while in precision mode



        double x = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double stickAngle = Math.atan2(direction ? -gamepad1.left_stick_y : gamepad1.left_stick_y, direction ? gamepad1.left_stick_x : -gamepad1.left_stick_x); // desired robot angle from the angle of stick
        double powerAngle = stickAngle - (Math.PI / 4); // conversion for correct power values
        double rightX = gamepad1.right_stick_x; // right stick x axis controls turning
        final double leftFrontPower = Range.clip(x * Math.cos(powerAngle)- rightX, -1.0, 1.0);
        final double leftRearPower = Range.clip(x * Math.sin(powerAngle) - rightX, -1.0, 1.0);
        final double rightFrontPower = Range.clip(x * Math.sin(powerAngle) + rightX, -1.0, 1.0);
        final double rightRearPower = Range.clip(x * Math.cos(powerAngle) +rightX, -1.0, 1.0);


        //move up Kush

if (gamepad2.right_trigger> .5){

    kush.setPosition(0);
    dinesh.setPosition(1);
}
if (gamepad2.left_trigger<.499999){
    kush.setPosition(1);
    dinesh.setPosition(0);


}
        if (gamepad2.x) {
            atul.setPosition(.5);
            saharsh.setPosition(.5);
            // satul[0].setPosition(1);
            // satul[1].setPosition(1);
        }

        if (gamepad2.dpad_up) {
            lift.setPower(1);
        }
        //Moving Down Kush
        if (gamepad2.dpad_down) {
            lift.setPower(-1);
        }

        //Stop Kush
        if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            lift.setPower(0);
        }
        //servodown Kush
        if (gamepad2.right_bumper) {
            kush.setPosition(1);
            dinesh.setPosition(0);
        }
        //servoup Kush
        else if (gamepad2.left_bumper) {
            kush.setPosition(0);
            dinesh.setPosition(1);
        }
        if (gamepad2.b) {
            atul.setPosition(.8);
            saharsh.setPosition(.8);
            // satul[0].setPosition(1);
            // satul[1].setPosition(1);
        }
        if (gamepad2.a) {
            atul.setPosition(.2);
            saharsh.setPosition(.2);
            // satul[0].setPosition(0);
            // satul[1].setPosition(0);
        }

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower(leftFrontPower * factor);
        leftBack.setPower(leftRearPower * factor);
        rightFront.setPower(rightFrontPower * factor);
        rightBack.setPower(rightRearPower * factor);


    }
    public double currentAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void correction(double error, double power, double targetHeading) {

        leftFront.setPower(power + pid.getCorrection(currentAngle() - targetHeading, runtime));
        rightFront.setPower(power - pid.getCorrection(currentAngle() - targetHeading, runtime));
        leftBack.setPower(power + pid.getCorrection(currentAngle() - targetHeading, runtime));
        rightBack.setPower(power - pid.getCorrection(currentAngle() - targetHeading, runtime));

        telemetry.addData("leftFront Power", leftFront.getPower());
        telemetry.addData("rightFront Power", rightFront.getPower());
        telemetry.addData("leftBack Power", leftBack.getPower());
        telemetry.addData("rightBack Power", rightBack.getPower());
        telemetry.addData("Current angle", currentAngle());
        telemetry.addData("Target Heading", targetHeading);
        telemetry.addData("Error Bibba", error);
        telemetry.update();

    }
//    public void lift (double distance, double power) throws InterruptedException{
//        int ticks = distance *250;
//        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        telemetry.addData("Ticks", ticks);
//
//        lift.setTargetPosition(ticks);
//        lift.setPower(power);
//
//        while (lift.isBusy()) {
//            telemetry.addData("ticks", ticks); telemetry.update();}
//        lift.setPower(0);
//    }


    }


