package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Transfer {
    public static double transferSpeed = 0.65; //every transfer power gets multiplied by this

    public DcMotorEx transfer;
    public CRServo transferServo;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transferServo = hardwareMap.get(CRServo.class, "transferServo");
    }

    //runs the transfer motor and servo together, negative runs them backwards
    public void run(double pwr) {
        transfer.setPower(pwr * transferSpeed);
        // positive power spins the servo counterclockwise
        transferServo.setPower(pwr * transferSpeed);
    }
}
