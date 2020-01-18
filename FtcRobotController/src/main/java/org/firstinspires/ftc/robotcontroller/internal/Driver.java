package org.firstinspires.ftc.robotcontroller.internal;
import  java.lang.*;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Driver", group = "")
public class Driver extends LinearOpMode{
  private DcMotor LeftFront;
  private DcMotor LeftBack;
  private DcMotor RightFront;
  private DcMotor RightBack;
  boolean toggleServo = false;
  private double multiplierWheels = 1;
  private double multiplierTurn = 1;
  private double multiplierArm = 1;
  private ColorSensor colorFront;
  private ColorSensor colorBottom;
  private double magnitude;
  private double highest;

  private double wheelSetA;
  private double wheelSetB;

  private DcMotor ArmMotor;
  private Servo ArmServo;

  long i = System.currentTimeMillis();
  /**
  * This function is executed when this Op Mode is selected from the Driver Station.
  */

  @Override
  public void runOpMode() {
    //map motors and servos
    LeftFront = hardwareMap.dcMotor.get("LeftFront");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightFront = hardwareMap.dcMotor.get("RightFront");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    colorFront = hardwareMap.colorSensor.get("colorFront");
    colorBottom = hardwareMap.colorSensor.get("colorBottom");
    ArmMotor = hardwareMap.dcMotor.get("ArmMotor");
    ArmServo = hardwareMap.servo.get("ArmServo");

//    //set encoder modes
//    LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//    LeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    LeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    RightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    RightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    //these are functions below
    call();
    set();

    while(opModeIsActive()) {
      //see below
      move();
      print();
    }
  }

  private void call() {
    // Call program, and devices
    waitForStart();
    ((DcMotorEx) LeftFront).setMotorEnable();
    ((DcMotorEx) LeftBack).setMotorEnable();
    ((DcMotorEx) RightFront).setMotorEnable();
    ((DcMotorEx) RightBack).setMotorEnable();
    ((DcMotorEx) ArmMotor).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
    RightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  private void move() {
    // Movement of robot with wheels
    // Left analog stick input

    magnitude = Math.sqrt(Math.pow(gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2));
    highest = Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x);

    if (highest == 0) {
      highest = 1;
    }

    wheelSetA = (gamepad1.left_stick_y - gamepad1.left_stick_x) * (1 - gamepad1.right_stick_x * multiplierTurn) * magnitude / highest;
    wheelSetB = (gamepad1.left_stick_y + gamepad1.left_stick_x) * (1 - gamepad1.right_stick_x * multiplierTurn) * magnitude / highest;

    LeftFront.setPower((wheelSetA + gamepad1.right_stick_x * multiplierTurn) * multiplierWheels);
    LeftBack.setPower((wheelSetB + gamepad1.right_stick_x * multiplierTurn) * multiplierWheels);
    RightFront.setPower((wheelSetB - gamepad1.right_stick_x * multiplierTurn) * multiplierWheels);
    RightBack.setPower((wheelSetA - gamepad1.right_stick_x * multiplierTurn) * multiplierWheels);

    //multiplier for wheel speed
    if(gamepad1.left_trigger > 0.5) {
      multiplierWheels = 0.8;
    } else {
      multiplierWheels = 0.4;
    }

    //multiplier for arm speed
    if(gamepad1.right_trigger > 0.5) {
      multiplierTurn = 1;
    } else {
      multiplierTurn = 0.6;
    }

    //lift control
    if(gamepad1.a){
      ArmMotor.setPower(0.3);
    }
    else if(gamepad1.b){
      ArmMotor.setPower(-0.3);
    }
    else{
      ArmMotor.setPower(0);
    }
    if(gamepad1.left_bumper && System.currentTimeMillis()-i>1000){
      toggleServo = !toggleServo;
      i = System.currentTimeMillis();
    }
    if(toggleServo) {
      ArmServo.setPosition(0.2);
    }
    else{
      ArmServo.setPosition(1);
    }
  }

  private void print() {
    // Prints data for debug purposes
    // telemetry.addData("GamePad2 Left Stick Y", gamepad2.left_stick_y);
    telemetry.addData("colorBottom blue", colorBottom.blue());
    telemetry.addData("colorFront Red", colorFront.red());
    telemetry.addData("colorFront Green", colorFront.green());
    telemetry.addData("colorFront Blue", colorFront.blue());
    telemetry.update();
  }
}
