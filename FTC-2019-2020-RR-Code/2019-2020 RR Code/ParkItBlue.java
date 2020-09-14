package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "ParkBlue", group = "Autonomous")
public class ParkItBlue extends LinearOpMode {
    private ElapsedTime runtime;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, lift;
    private DcMotorEx[] motors = {leftFront, leftBack, rightFront, rightBack,lift};
    private Servo dinesh, kush;
    private BNO055IMU imu;
    private PID pid = new PID(.022, 0.0003, 0.0022);
    private PID strafe = new PID(.02, .0003, .002);
    private final double C = 537 / (Math.PI * 4 * (5.0 / 6)), STRAFE_COEFFICIENT = 1.12943302;

    public void runOpMode() {
        initialize();

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        try {
            dinesh.setPosition(0);
            kush.setPosition(1);
            pause(1);
            strafe(42,.5,"Left",0);


        } catch (Exception e) {
        }
    }

    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDrive");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftRearDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDrive");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightRearDrive");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        dinesh = hardwareMap.servo.get("servoright");
        kush = hardwareMap.servo.get("servoleft");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        dinesh.setDirection(Servo.Direction.FORWARD);
        motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack,lift};

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        runtime = new ElapsedTime();
    }

    public void move(double targetHeading, double power, int ticks) {

        ticks = ticks * 268;
        setTargetPosition(ticks);
        reset();
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            correction(currentAngle() - targetHeading, power, targetHeading);
        }

        halt();
    }

    public double currentAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void reset() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void setTargetPosition(int targetPosition) {
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(targetPosition);
        }
    }

    public void halt() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }


    public void strafe(double distance, double power, String dir, double targetHeading) throws InterruptedException {
        int ticks = (int) (distance * 537 / (Math.PI * 4));
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (dir.equals("right")) { // If statement
            leftFront.setTargetPosition(ticks); // sets position of the target posintion of left friong motor
            leftFront.setPower(power);
            leftBack.setTargetPosition(-ticks);
            leftBack.setPower(-power);
            rightFront.setTargetPosition(-ticks);
            rightFront.setPower(-power);
            rightBack.setTargetPosition(ticks);
            rightBack.setPower(power);
        } else if (dir.equals("left")) {
            leftFront.setTargetPosition(-ticks);
            leftFront.setPower(-power);
            leftBack.setTargetPosition(ticks);
            leftBack.setPower(power);
            rightFront.setTargetPosition(ticks);
            rightFront.setPower(power);
            rightBack.setTargetPosition(-ticks);
            rightBack.setPower(-power);
        }
        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {

            strafecorrection(currentAngle()-targetHeading,power,targetHeading,dir);
        }
        halt();
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
    public void strafecorrection ( double error, double power, double targetHeading,String dir){
        double target = targetHeading;
        double current = currentAngle();

        //if the spline motion is backwards, the target must be flipped 180 degrees in order to match with spline.getAngle()


        //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
        if (targetHeading < -135 && currentAngle() > 135) {
            target = targetHeading + 360.0;
        }
        else if (targetHeading > 135 && currentAngle() < -135) {
            current = currentAngle() + 360.0;
        }

        if (target > 180) {
            target-=360;
        }
        else if (target < -180) {
            target += 360;
        }

        if (dir.equals("left")) {

            leftFront.setPower(Range.clip(-power + strafe.getCorrection(current - target, runtime),-1.0, 1.0));
            rightFront.setPower(Range.clip(power - strafe.getCorrection(current - target, runtime), -1.0, 1.0));
            leftBack.setPower(Range.clip(power + strafe.getCorrection(current - target, runtime), -1.0, 1.0));
            rightBack.setPower(Range.clip(-power - strafe.getCorrection(current - target, runtime), -1.0, 1.0));
        }
        else if (dir.equals("right")) {
            leftFront.setPower(Range.clip(power + strafe.getCorrection(current - target, runtime), -1.0, 1.0));
            rightFront.setPower(Range.clip(-power - strafe.getCorrection(current - target, runtime), -1.0, 1.0));
            leftBack.setPower(Range.clip(-power + strafe.getCorrection(current - target, runtime), -1.0, 1.0));
            rightBack.setPower(Range.clip(power - strafe.getCorrection(current - target, runtime), -1.0, 1.0));
        }
//        leftFront.setPower(power + pid.getCorrection(currentAngle() - targetHeading, runtime));
//        rightFront.setPower(power - pid.getCorrection(currentAngle() - targetHeading, runtime));
//        leftBack.setPower(power + pid.getCorrection(currentAngle() - targetHeading, runtime));
//        rightBack.setPower(power - pid.getCorrection(currentAngle() - targetHeading, runtime));

        telemetry.addData("leftFront Power", leftFront.getPower());
        telemetry.addData("rightFront Power", rightFront.getPower());
        telemetry.addData("leftBack Power", leftBack.getPower());
        telemetry.addData("rightBack Power", rightBack.getPower());
        telemetry.addData("Current angle", currentAngle());
        telemetry.addData("targetHeading ", targetHeading);
        telemetry.addData("ErrorBibba ", error);
        telemetry.update();

    }
    public void setLift(double distance, double power) throws InterruptedException {
        int ticks = (int) (distance * 537 / (Math.PI * 4));
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy()) {
            heartbeat();
//            strafecorrection(currentAngle()-targetHeading,power,targetHeading,dir);
        }
        halt();

    }



    public void pause ( double time) throws InterruptedException {
        double pause = runtime.time();
        while (runtime.time() - pause < time) {
            heartbeat();
            telemetry.addData("Paused ", time);
            telemetry.update();
        }

    }

    private void heartbeat () throws InterruptedException {
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }


}


