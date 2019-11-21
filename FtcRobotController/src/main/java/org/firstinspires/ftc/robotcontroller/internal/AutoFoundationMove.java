package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "AutoFoundationMove", group = "")
  public class AutoFoundationMove extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor LeftBack;
    private DcMotor RightFront;
    private DcMotor RightBack;
    private DcMotor ClawArm;
    private DcMotor ClawArm2;
    private Servo servo;
    private Servo servo2;

    private float multiplier = 1;

    private double leftFront = 1;
    private double rightFront = 1;
    private double leftBack = 1;
    private double rightBack = 1;

  /**
  * This function is executed when this Op Mode is selected from the Driver Station.
  */

  @Override
  public void runOpMode() {
    LeftFront = hardwareMap.dcMotor.get("LeftFront");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    RightFront = hardwareMap.dcMotor.get("RightFront");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    ClawArm = hardwareMap.dcMotor.get("ClawArm");
    ClawArm2 = hardwareMap.dcMotor.get("ClawArm2");
    servo = hardwareMap.get(Servo.class, "Grab");
    servo2 = hardwareMap.get(Servo.class, "Grab2");

    //these are functions below
    call();
    set();

    while (opModeIsActive()) {
      //see below
      move(1, 0, 0, 1);

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
    ((DcMotorEx) ClawArm).setMotorEnable();
    ((DcMotorEx) ClawArm2).setMotorEnable();
  }

  private void set() {
    // Set direction of devices
    LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    RightBack.setDirection(DcMotorSimple.Direction.REVERSE);
    ClawArm.setDirection(DcMotorSimple.Direction.FORWARD);
    ClawArm2.setDirection(DcMotorSimple.Direction.FORWARD);
  }

  private void move(float y, float x, float rotation, float duration) {
    LeftFront.setPower(((y - x) * 0.5 - rotation * 0.3) * leftFront);
    LeftBack.setPower(((y + x) * 0.5 - rotation * 0.3) * rightFront);
    RightFront.setPower(((y + x) * 0.5 + rotation * 0.3) * leftBack);
    RightBack.setPower(((y - x) * 0.5 + rotation * 0.3) * rightBack);
    sleep(duration*1000);
  }

  private void stdFlip(float stdFlipPower) {
    ClawArm.setPower(stdFlipPower);
  }

  private void stdHeight(float stdVerticalPower) {
    ClawArm2.setPower(stdVerticalPower);
  }

  private void std(boolean stdPower) {
    if(stdPower == true){
      servo.setPosition(1);
      servo2.setPosition(0);
    } else {
      servo.setPosition(0);
      servo2.setPosition(0.8);
    }
  }

  private void print() {
    // Prints data for debug purposes
    telemetry.addData("gamepad1 left stick x", gamepad1.left_stick_x);
    telemetry.update();
  }
}
