package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Transfer {
    public static double transferSpeed = 1; //every transfer power gets multiplied by this

    //speed gate: only feed a ball when the flywheel is back up to speed
    //edit these live in the dashboard config tab
    public static boolean speedGate = false; //false feeds whenever the button is held, like before
    public static double stopTolerance = 30; //ticks/sec below target that pauses feeding (a ball just went through)
    public static double readyTolerance = 20; //ticks/sec below target where feeding is allowed again
    public static double settleTime = 0.05; //seconds the flywheel must stay ready before the next ball goes

    private boolean gateOpen = false;
    private final ElapsedTime readyTimer = new ElapsedTime();

    public DcMotorEx transfer;
    public CRServo transferServo;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
    }

    //call every loop instead of run() when shooting
    //feeds only while the flywheel is close to its target speed, returns true while feeding
    public boolean feed(boolean wantToShoot, double velocity, double targetVelocity) {
        if (!wantToShoot || targetVelocity <= 0) {
            gateOpen = false;
            run(0);
            return false;
        }
        if (!speedGate) {
            run(1);
            return true;
        }

        double drop = targetVelocity - velocity;
        if (gateOpen) {
            //a ball hit the wheel and slowed it down, hold the next ball back
            if (drop > stopTolerance) {
                gateOpen = false;
                readyTimer.reset();
            }
        } else {
            if (drop > readyTolerance) {
                //still recovering, restart the settle timer
                readyTimer.reset();
            } else if (readyTimer.seconds() >= settleTime) {
                gateOpen = true;
            }
        }

        run(gateOpen ? 1 : 0);
        return gateOpen;
    }

    //runs the transfer motor and servo together, negative runs them backwards
    public void run(double pwr) {
        transfer.setPower(pwr * transferSpeed);
        // positive power spins the servo counterclockwise
        transferServo.setPower(pwr * transferSpeed);
    }
}