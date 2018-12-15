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
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Autonomous Mode Depot", group = "")
public class AutonomousModeA extends LinearOpMode {
  private DcMotor LeftWheel;
  private DcMotor RightWheel;
  private DcMotor DefaultArm;
  private Servo LeftClaw;
  private Servo RightClaw;
  private DcMotor InceptionArm;
  private ColorSensor ColorSensor;
  private DistanceSensor DistanceSensor;

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
    DistanceSensor = hardwareMap.get(DistanceSensor.class, "Color Sensor");
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
    InceptionArm.setPower(-0.15);
    // Move 5.66 feet
    RightWheel.setPower(-0.83);
    LeftWheel.setPower(-0.83);
    sleep(1100);
    LeftWheel.setPower(0.5);
    RightWheel.setPower(-0.5);
    sleep(1475);
    RightWheel.setPower(-0.6);
    LeftWheel.setPower(-0.6);
    sleep(1600);

    for(int i = 0; i < 3; i++) {
      //while(hsvValues[0] > 130) {
      LeftWheel.setPower(0);
      RightWheel.setPower(0);
      sleep(500);
      Color.RGBToHSV((int) (ColorSensor.red() * 255),
      (int) (ColorSensor.green() * 255),
      (int) (ColorSensor.blue() * 255),
      hsvValues);
      double colorInput = hsvValues[0];
      telemetry.addData("Hue: "+i, hsvValues[0]);
      telemetry.addData("Distance (cm)",
              String.format(Locale.US, "%.02f", DistanceSensor.getDistance(DistanceUnit.CM)));
      //}
      //movement between silvers and golds
      telemetry.update();
      //if gold knock it outs of here's
      if(colorInput < 128 || i == 2) {
        RightWheel.setPower(-0.25);
        LeftWheel.setPower(-0.25);
        sleep(200);
        RightWheel.setPower(-0.5);
        LeftWheel.setPower(0.5);
        sleep(1333);
        break;
      } else {
        LeftWheel.setPower(0.5);
        RightWheel.setPower(0.5);
        sleep(1250);
      }
    }
  }
  // Deploy team marker
  /*
  InceptionArm.setPower(0.3);
  LeftClaw.setPosition(1);
  RightClaw.setPosition(1);
  InceptionArm.setPower(-0.1);
  sleep(300);
  InceptionArm.setPower(0);
  LeftWheel.setPower(0.5);
  sleep(666);
  RightWheel.setPower(0.75);
  LeftWheel.setPower(0.75);
  sleep(1000);
  LeftWheel.setPower(0);
  RightWheel.setPower(0);
  // sleep(500);
  // 45 degree right turn
  // LeftWheel.setPower(0.5);
  // RightWheel.setPower(-0.5);
  // sleep(500);
  // // Move 7 feet
  // LeftWheel.setPower(1);
  // RightWheel.setPower(1);
  // sleep(4200);
  }
  */
}
