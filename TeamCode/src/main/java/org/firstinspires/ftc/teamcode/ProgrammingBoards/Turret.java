package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

public class Turret {
    public Limelight3A limelight;
    public DcMotorEx turret;

    public Turret(HardwareMap hardwareMap) {
//        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
//        limelight.pipelineSwitch(0);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        limelight.start();

    }

    public static double turretSpeed = -0.2;

    // positive power = counterclockwise
    public void spinTurret(double pwr) {
        turret.setPower(pwr * turretSpeed);
    }

//    public double returnTx(){
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()){
//            return result.getTx();
//        }
//        else{
//            return 1000;
//        }
//    }
}