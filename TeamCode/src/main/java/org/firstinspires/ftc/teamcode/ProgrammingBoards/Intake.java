package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    public DcMotorEx intake;
    public Servo flowerIntake;
    //increasing position is counterclockwise
    public static double flowerUpPosition = 0.6;
    public static double flowerDownPosition = 0;
    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flowerIntake = hardwareMap.get(Servo.class, "flowerIntake");
        flowerIntake.setPosition(flowerUpPosition);
    }
    public void flowerUp(){
        flowerIntake.setPosition(flowerUpPosition);
    }
    public void flowerDown(){
        flowerIntake.setPosition(flowerDownPosition);
    }
    public void runIntake(double pwr){
        intake.setPower(pwr);
    }

}
