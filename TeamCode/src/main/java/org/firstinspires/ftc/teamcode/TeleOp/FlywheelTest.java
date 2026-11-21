package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Transfer;

import com.qualcomm.robotcore.util.ElapsedTime;

//only the flywheel, intake and transfer run, for tuning the flywheel pidf
@Config
@TeleOp(name = "FlywheelTest")
public class FlywheelTest extends LinearOpMode {
    //edit live in the dashboard config tab, flywheel pidf is under Flywheel
    public static double testVelocity = 780; //ticks per second
    public static double velocityStep = 20; //dpad up and down change speed by this
    public static double maxTestVelocity = 1500; //speed never goes above this

    @Override
    public void runOpMode() throws InterruptedException {

        //sends telemetry to both driver station and ftc dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        ElapsedTime loopTimer = new ElapsedTime(); //loop timer

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            //dpad up and down change the speed, one step per press
            if (gamepad1.dpad_up && !lastDpadUp) {
                testVelocity += velocityStep;
            }
            if (gamepad1.dpad_down && !lastDpadDown) {
                testVelocity -= velocityStep;
            }
            lastDpadUp = gamepad1.dpad_up;
            lastDpadDown = gamepad1.dpad_down;
            testVelocity = Math.max(0, Math.min(maxTestVelocity, testVelocity));

            double error = flywheel.returnVel() - testVelocity;
            double percentError = (((flywheel.returnVel() - testVelocity)/testVelocity) * 100);

            flywheel.update(testVelocity, battery.getVoltage());

            //right bumper runs transfer, left bumper runs intake
            transfer.run(gamepad1.right_bumper ? 1 : 0);
            intake.runIntake(gamepad1.left_bumper ? 1 : 0);

            telemetry.addData("velocity", flywheel.returnVel());
            telemetry.addData("targetVelocity", testVelocity);
            telemetry.addData("percentError", percentError);
            telemetry.addData("error", error);
            telemetry.addData("Flywheel power", flywheel.returnPower());
            telemetry.addData("Battery voltage", battery.getVoltage());
            telemetry.addData("loopMs", loopTimer.milliseconds());
            loopTimer.reset();
            telemetry.update();
        }
    }
}
