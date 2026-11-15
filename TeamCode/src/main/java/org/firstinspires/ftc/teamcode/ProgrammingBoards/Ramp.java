package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Ramp {
    //edit these live in the dashboard config tab
    public static double upPosition = 0.6; //servo position where the ramp points straight up, same as flowerUpPosition
    public static double downDegrees = 180; //degrees the ramp moves down from straight up
    public static double servoRange = 300; //degrees the servo turns from 0 to 1
    public static double downDirection = -1; //flip to 1 if the ramp goes the wrong way

    public Servo ramp;
    private boolean down = false;

    public Ramp(HardwareMap hardwareMap) {
        ramp = hardwareMap.get(Servo.class, "flowerIntake");
        up();
    }

    public void up() {
        down = false;
        ramp.setPosition(upPosition);
    }

    public void down() {
        down = true;
        ramp.setPosition(upPosition + downDirection * downDegrees / servoRange);
    }

    //switches between up and down
    public void toggle() {
        if (down) {
            up();
        } else {
            down();
        }
    }
}
