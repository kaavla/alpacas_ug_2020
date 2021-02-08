package org.firstinspires.ftc.teamcode.TestBot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp(name = "Manav Bot", group = "Tests")
//@Disabled
public class MKBot extends LinearOpMode {


    private DcMotorEx m1, m2;
    public Servo kick = null;
    public Servo turn = null;


    @Override
    public void runOpMode() {


        // Important Step 1:  Make sure you use DcMotorEx when you instantiate your motors.
        m1 = hardwareMap.get(DcMotorEx.class, "m1");  // Configure the robot to use these 4 motor names,
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        kick = hardwareMap.get(Servo.class, "kick");
        turn = hardwareMap.get(Servo.class, "turn");

    // Important Step 2: Get access to a list of Expansion Hub Modules to enable changing caching methods.

        ElapsedTime timer = new ElapsedTime();

        waitForStart();


        timer.reset();
        double pos = 0.0;

        while (opModeIsActive()) {
            if (gamepad1.y) {
                m1.setPower(0.7);
                m2.setPower(-0.7);
            } else if (gamepad1.a) {
                m1.setPower(0);
                m2.setPower(0);
            } else if (gamepad1.left_bumper){
                pos = pos + 0.1;
                if (pos >= 0.9) pos = 0.9;
                turn.setPosition(pos);
                sleep(500);
            } else if (gamepad1.right_bumper) {
                pos = pos - 0.1;
                if (pos <= 0) pos = 0.0;
                turn.setPosition(pos);
                sleep(500);
            } else if (gamepad1.b) {
                kick.setPosition(0.1);
            }else if (gamepad1.x) {
                for (int i = 0; i < 3; i++) {
                    kick.setPosition(0.1);
                    sleep(250);
                    kick.setPosition(0.35);
                    sleep(250);
                }
                kick.setPosition(0.1);
            }else if (gamepad1.right_stick_button) {
                kick.setPosition(0.35);
            }
        }
    }
}

