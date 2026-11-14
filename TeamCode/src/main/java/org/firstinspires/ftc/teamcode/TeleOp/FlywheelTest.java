package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;

//only the flywheel, intake and transfer run, for tuning the flywheel pidf
@Config
@TeleOp(name = "FlywheelTest")
public class FlywheelTest extends LinearOpMode {
    //edit live in the dashboard config tab, flywheel pidf is under Flywheel
    public static double testVelocity = 1465; //ticks per second

    @Override
    public void runOpMode() throws InterruptedException {

        //sends telemetry to both driver station and ftc dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            flywheel.update(testVelocity, battery.getVoltage());

            //intake and transfer always run
            intake.runIntake(1);
            intake.runTransfer(1);

            telemetry.addData("Velocity", flywheel.returnVel());
            telemetry.addData("targetVelocity", testVelocity);
            telemetry.addData("Flywheel power", flywheel.returnPower());
            telemetry.addData("Battery voltage", battery.getVoltage());
            telemetry.update();
        }
    }
}
