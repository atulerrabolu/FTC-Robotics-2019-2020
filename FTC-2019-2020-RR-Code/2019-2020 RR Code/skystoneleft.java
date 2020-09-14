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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Autonomous(name = "skystonecheckblue", group = "Autonomous")
public class skystoneleft extends LinearOpMode {
   private final double C = 537 / (Math.PI * 4 * (5.0 / 6)), STRAFE_COEFFICIENT = 1.12943302;



   public void runOpMode() {
       position = 0;
       initialize();
       telemetry.addData("Status", "Initialized Kush and Atul ");
       telemetry.update();


       waitForStart();

       try {


           saharsh.setPosition(1);
           atul.setPosition(1);
           dinesh.setPosition(1);
           kush.setPosition(0);

           //first block
           pause(1);
           move(0, -.5, -370/17.0);
           pause(.1);
           strafe(49, .3, "right", 0);
           pause(.1);
           move(0, .3, 6);
           pause(.1);
           atul.setPosition(0);
           saharsh.setPosition(0);
           pause(1);
           strafe(25, .3, "left", 0);
           pause(.1);
           move(0, .5, 590/17.0);

           //second
           pause(.1);
           atul.setPosition(1);
           saharsh.setPosition(1);
           pause(1);
           move(0, -.4, -515/17.0);
           pause(.1);
           setEncoders();
           while (currentAngle() > -160) {
               heartbeat();
               telemetry.addData("Status", currentAngle());
               telemetry.update();
               turn("right", .3);
           }
           halt();
           pause(.1);
           strafe(16, .3, "left", -175);
           pause(.1);
           move(-175, .2, 7);
           pause(.1);
           atul.setPosition(0);
           saharsh.setPosition(0);
           pause(1);
           strafe(23, .3, "right", -175);
           pause(.1);
           setEncoders();
           while (Math.abs(currentAngle()) > 40) {
               turn("left", .7);
               telemetry.addData("Status", currentAngle());
               telemetry.update();
           }
           halt();
           pause(.1);
           move(0, .3, 610/17.0);
           pause(.1);
           atul.setPosition(1);
           saharsh.setPosition(1);
           pause(1);
           move(0, -.5, -10/1.7);
           pause(.1);
           strafe(23, .3, "right", 0);
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
       saharsh = hardwareMap.servo.get("lift1");
       atul = hardwareMap.servo.get("lift2");

       leftFront.setDirection(DcMotor.Direction.REVERSE);
       leftBack.setDirection(DcMotor.Direction.REVERSE);
       dinesh.setDirection(Servo.Direction.FORWARD);
       saharsh.setDirection(Servo.Direction.FORWARD);
       atul.setDirection(Servo.Direction.REVERSE);
       motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack, lift};

       imu = hardwareMap.get(BNO055IMU.class, "imu");
       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
       parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
       imu.initialize(parameters);

       runtime = new ElapsedTime();
   }

   public void initVuforia() {
       cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
       parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
       ab.initVuforia(parameters);
   }

   public void move(double targetHeading, double power, double ticks) {

       ticks = ticks * 537 / (Math.PI * 4 * (5.0 / 6));
       int realticks = 0;
       realticks = (int) ticks;
       setTargetPosition(realticks);
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
//            heartbeat();
           strafecorrection(currentAngle() - targetHeading, power, targetHeading, dir);
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

   public void strafecorrection(double error, double power, double targetHeading, String dir) {

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
   /*
       telemetry.addData("leftFront Power", leftFront.getPower());
       telemetry.addData("rightFront Power", rightFront.getPower());
       telemetry.addData("leftBack Power", leftBack.getPower());
       telemetry.addData("rightBack Power", rightBack.getPower());
       telemetry.addData("Current angle", currentAngle());
       telemetry.addData("targetHeading ", targetHeading);
       telemetry.addData("ErrorBibba ", error);
       telemetry.update();*/

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
   public void turn(double targetHeading, double power, String direction) throws InterruptedException {
       double target = targetHeading;
       double current = currentAngle();

       if (targetHeading < -135 && currentAngle() > 135) {
           target = targetHeading + 360.0;
       }
       else if (targetHeading > 135 && currentAngle() < -135) {
           current = currentAngle() + 360.0;
       }
       setEncoders();
       while (Math.abs(target - current) < 10) {
           heartbeat();

           turn(direction, power);

           if (targetHeading < -135 && currentAngle() > 135) {
               target = targetHeading + 360.0;
           }
           else if (targetHeading > 135 && currentAngle() < -135) {
               current = currentAngle() + 360.0;
           }
       }

       halt();
   }
   public void turn(String direction, double power) {
       if (direction.equals("left")) {
           leftFront.setPower(-power);
           leftBack.setPower(-power);
           rightFront.setPower(power);
           rightBack.setPower(power);
       } else if (direction.equals("right")) {
           leftFront.setPower(power);
           leftBack.setPower(power);
           rightFront.setPower(-power);
           rightBack.setPower(-power);

       }
       telemetry.addData("ErrorBibba ", currentAngle());
   }

   public void setEncoders() {
       leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }


   public void pause(double time) throws InterruptedException {
       double pause = runtime.time();
       while (runtime.time() - pause < time) {
           heartbeat();
           telemetry.addData("Paused ", time);
           telemetry.update();
       }

   }

   private void heartbeat() throws InterruptedException {
       if (!opModeIsActive()) {
           throw new InterruptedException();
       }
   }
}


