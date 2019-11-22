package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "ServoTesting", group = "")
public class ServoTesting extends LinearOpMode {
    private Servo servo;
    private Servo servo2;
    private double pos = 0;
    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "Grab");
        servo2 = hardwareMap.get(Servo.class, "Grab2");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                pos = pos + 0.01;
            } else if (gamepad1.b) {
                pos = pos - 0.01;
            }
            servo.setPosition(0.5+pos);
            servo2.setPosition(0.5-pos);
            telemetry.addData("Position", pos);
            telemetry.update();
        }
    }
}