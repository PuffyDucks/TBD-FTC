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

@Autonomous(name = "Test", group = "")
public class Test extends LinearOpMode {
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

    waitForStart();

    while(opModeIsActive()){
      InceptionArm.setPower(0.3);
    }
  }
}
