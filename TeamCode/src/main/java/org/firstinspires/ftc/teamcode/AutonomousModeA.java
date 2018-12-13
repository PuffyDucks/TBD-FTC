    package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "Autonomous Mode A", group = "")
public class AutonomousModeA extends LinearOpMode {
  private DcMotor LeftWheel;
  private DcMotor RightWheel;
  private DcMotor DefaultArm;
  private Servo LeftClaw;
  private Servo RightClaw;
  private DcMotor InceptionArm;
  private ColorSensor ColorSensor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    DefaultArm = hardwareMap.dcMotor.get("Default Arm");
    InceptionArm = hardwareMap.dcMotor.get("Inception Arm");
    LeftWheel = hardwareMap.dcMotor.get("Left Wheel");
    RightWheel = hardwareMap.dcMotor.get("Right Wheel");
    LeftClaw = hardwareMap.servo.get("Left Claw");
    RightClaw = hardwareMap.servo.get("Right Claw");
    ColorSensor = hardwareMap.colorSensor.get("Color Sensor");

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;

    ((DcMotorEx) DefaultArm).setMotorEnable();
    ((DcMotorEx) InceptionArm).setMotorEnable();
    ((DcMotorEx) LeftWheel).setMotorEnable();
    ((DcMotorEx) RightWheel).setMotorEnable();
    DefaultArm.setDirection(DcMotorSimple.Direction.FORWARD);
    InceptionArm.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    RightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftClaw.setDirection(Servo.Direction.REVERSE);
    RightClaw.setDirection(Servo.Direction.FORWARD);
    //LANDER LAND HERE
    waitForStart();
    // Move 5.66 feet
    RightWheel.setPower(0.5);
    LeftWheel.setPower(0.5);
    sleep(1000);
    LeftWheel.setPower(0.25);
    RightWheel.setPower(0.25);
    sleep(1000);

    for(int i = 0; i < 3; i++) {
      //while(hsvValues[0] > 130) {
      sleep(1000);
        Color.RGBToHSV((int) (ColorSensor.red() * 255),
        (int) (ColorSensor.green() * 255),
        (int) (ColorSensor.blue() * 255),
        hsvValues);
        double colorInput = hsvValues[0];
        telemetry.addData("Hue", hsvValues[0]);
      //}
      //movement between silvers and golds
      telemetry.update();
      sleep(1000);
      //if gold knock it outs of here's
      if(colorInput < 100 || i == 2) {
        RightWheel.setPower(-0.5);
        LeftWheel.setPower(0.5);
        sleep(600);
        i = 3;
        RightWheel.setPower(0.5);
        LeftWheel.setPower(0.5);
        sleep(300);
      } else {
        LeftWheel.setPower(0.5);
        RightWheel.setPower(0.5);
        sleep(650);
        LeftWheel.setPower(0);
        RightWheel.setPower(0);
      }
    }
    LeftWheel.setPower(0.5);
    RightWheel.setPower(0.5);
    sleep(3394);
    // Deploy team marker
    sleep(500);
    LeftClaw.setPosition(0.8);
    RightClaw.setPosition(0.8);
    sleep(500);
    LeftClaw.setPosition(0.3);
    RightClaw.setPosition(0.3);
    sleep(500);
    // 45 degree right turn
    LeftWheel.setPower(0.5);
    RightWheel.setPower(-0.5);
    sleep(500);
    // Move 7 feet
    LeftWheel.setPower(1);
    RightWheel.setPower(1);
    sleep(4200);


  }
}
