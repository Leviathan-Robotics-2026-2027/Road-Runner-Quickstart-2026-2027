package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Flywheel {
    //edit these live in the dashboard config tab
    //kV, kP, kD are divided by 10000 in the math so they stay whole-ish numbers
    public static double kV = 5.4; //base power for a speed
    public static double kS = 1; //percent power to overcome friction
    public static double kP = 75; //push back against speed error
    public static double kD = 3.5; //damps fast changes
    public static double nominalVoltage = 12; //voltage kV and kS are tuned for

    private static final double gainScale = 10000;

    public DcMotorEx flywheel;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastPower = 0;

    public Flywheel(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        //we do the pidf ourselves, the encoder is still read for velocity
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //call every loop
    public void update(double targetVelocity, double batteryVoltage){
        double dt = timer.seconds();
        timer.reset();
        //skip the first loop and any long pauses so d doesn't spike
        if (dt <= 0 || dt > 0.1) {
            dt = 0;
        }

        if (targetVelocity <= 0) {
            lastError = 0;
            lastPower = 0;
            flywheel.setPower(0);
            return;
        }

        double error = targetVelocity - returnVel();

        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        //feedforward is scaled to the battery so the same speed needs the same numbers on any charge
        double feedforward = (kV / gainScale * targetVelocity + kS / 100) * nominalVoltage / batteryVoltage;
        double feedback = (kP * error + kD * derivative) / gainScale;

        //never drive backwards, just coast down if too fast
        lastPower = Math.max(0, Math.min(1, feedforward + feedback));
        flywheel.setPower(lastPower);
    }

    public double returnVel(){
        return flywheel.getVelocity();
    }

    public double returnPower(){
        return lastPower;
    }
}
