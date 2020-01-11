package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ScrappyDriving", group = "")
public class ScrappyDriving extends LinearOpMode{
  private DcMotor FrontLeft;
  private DcMotor FrontRight;
  private DcMotor BackLeft;
  private DcMotor BackRight;

  @Override
  public void runOpMode() {
    //Map motors
    FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
    FrontRight = hardwareMap.dcMotor.get("FrontRight");
    BackLeft = hardwareMap.dcMotor.get("BackLeft");
    BackRight = hardwareMap.dcMotor.get("BackRight");

    //Wait for start
    waitForStart();

    //Call program, and devices
    ((DcMotorEx) FrontLeft).setMotorEnable();
    ((DcMotorEx) BackLeft).setMotorEnable();
    ((DcMotorEx) FrontLeft).setMotorEnable();
    ((DcMotorEx) BackRight).setMotorEnable();

    //Set direction of devices
    FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

    while(opModeIsActive()) {
      //Take inputs and move motors
      move();
      //Send back telemetry data
      print();
    }
  }

  private void move() {

  }

  private void print() {

  }
}
