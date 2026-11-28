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
    public static double shotVelocity = 765; //flywheel speed every auton and teleop uses, ticks per second
    public static double kV = 8.6; //base power for a speed
    public static double kS = 0; //percent power to overcome friction
    public static double kP = 75; //push back against speed error
    public static double kD = 0.4; //dam ps fast changes
    public static double nominalVoltage = 12; //voltage kV and kS are tuned for
    public static boolean batteryScaling = false; //true scales the feedforward to the battery voltage
    public static double voltageSmoothing = 0.05; //0 to 1, how fast the battery reading follows changes, low ignores quick dips from other motors

    private static final double gainScale = 10000;

    public DcMotorEx flywheel;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastPower = 0;
    private double smoothVoltage = -1; //battery voltage with the quick dips smoothed out

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

        double feedforward = kV / gainScale * targetVelocity + kS / 100;
        if (batteryScaling) {
            //intake and other motors make the reading jump, so follow it slowly
            if (smoothVoltage < 0) {
                smoothVoltage = batteryVoltage;
            } else {
                smoothVoltage += voltageSmoothing * (batteryVoltage - smoothVoltage);
            }
            //same speed needs the same numbers on any charge
            feedforward *= nominalVoltage / smoothVoltage;
        }
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
