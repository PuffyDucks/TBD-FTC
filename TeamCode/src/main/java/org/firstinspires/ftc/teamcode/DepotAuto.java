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

@Autonomous(name = "DepotAuto", group = "")
public class DepotAuto extends LinearOpMode {
  private DcMotor LeftWheel;
  private DcMotor RightWheel;
  private DcMotor LanderArm;
  private DcMotor LanderArm2;
  private DcMotor Succ;
  private DcMotor SuccArm;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    LeftWheel = hardwareMap.dcMotor.get("LeftWheel");
    RightWheel = hardwareMap.dcMotor.get("RightWheel");
    LanderArm = hardwareMap.dcMotor.get("LanderArm");
    LanderArm2 = hardwareMap.dcMotor.get("LanderArm2");
    Succ = hardwareMap.dcMotor.get("Succ");
    SuccArm = hardwareMap.dcMotor.get("SuccArm");

    RightWheel.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    LanderArm.setPower(-0.5);
    LanderArm2.setPower(0.5);
    sleep(2250);
    LanderArm.setPower(0);
    LanderArm2.setPower(0);

    LeftWheel.setPower(1);
    RightWheel.setPower(1);
    sleep(100);

    LeftWheel.setPower(-1);
    RightWheel.setPower(1);
    sleep(200);

    LeftWheel.setPower(-1);
    RightWheel.setPower(-1);
    sleep(600);

    LeftWheel.setPower(1);
    RightWheel.setPower(-1);
    sleep(350);

    RightWheel.setPower(-1);
    LeftWheel.setPower(-1);
    sleep(1000);
    RightWheel.setPower(0);
    LeftWheel.setPower(0);

    Succ.setPower(1);
    sleep(1000);
    Succ.setPower(0);
  }
}
