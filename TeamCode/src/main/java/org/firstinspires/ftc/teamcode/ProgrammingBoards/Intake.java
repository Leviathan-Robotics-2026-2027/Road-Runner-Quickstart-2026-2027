package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    //edit these live in the dashboard config tab
    public static double rampUpPosition = 0.5; //servo position where the ramp points straight up
    public static double rampDownDegrees = 150; //degrees the ramp moves down from straight up
    public static double rampServoRange = 300; //degrees the servo turns from 0 to 1
    public static double rampDownDirection = -1; //flip to 1 if the ramp goes the wrong way

    public DcMotorEx intake;
    public Servo ramp;
    private boolean rampIsDown = false;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        ramp = hardwareMap.get(Servo.class, "flowerIntake");
        rampUp(); //starts straight up
    }

    public void runIntake(double pwr){
        intake.setPower(pwr);
    }

    public void rampUp(){
        rampIsDown = false;
        ramp.setPosition(rampUpPosition);
    }

    public void rampDown(){
        rampIsDown = true;
        ramp.setPosition(rampUpPosition + rampDownDirection * rampDownDegrees / rampServoRange);
    }

    //switches the ramp between up and down
    public void toggleRamp(){
        if (rampIsDown) {
            rampUp();
        } else {
            rampDown();
        }
    }
}
