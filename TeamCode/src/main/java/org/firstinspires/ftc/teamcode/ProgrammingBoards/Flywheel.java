package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flywheel {
    public DcMotorEx flywheel;
    public Flywheel(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runFlywheelVel(double speed){
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double kP = 1;
        double kI = 1;
        double kD = 1;
        double kF = 1;

        double integralSum = 0;
        double lastError = 0;

        ElapsedTime timer = new ElapsedTime();

        double targetVelocity = speed * 1600;
        double currentVelocity = flywheel.getVelocity();

        double error = targetVelocity - currentVelocity;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * targetVelocity);


        flywheel.setVelocity(output);


        lastError = error;
        timer.reset();
    }

    public double returnVel(){
        return flywheel.getVelocity();
    }

}
