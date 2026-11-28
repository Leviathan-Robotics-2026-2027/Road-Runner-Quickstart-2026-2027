package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Transfer;

import com.qualcomm.robotcore.util.ElapsedTime;

//only the flywheel, intake and transfer run, for tuning the flywheel pidf
@Config
@TeleOp(name = "FlywheelTest")
public class FlywheelTest extends LinearOpMode {
    //edit live in the dashboard config tab, flywheel pidf is under Flywheel
    public static double velocityStep = 5; //dpad up and down change speed by this
    public static double maxTestVelocity = 1000; //speed never goes above this

    @Override
    public void runOpMode() throws InterruptedException {

        //sends telemetry to both driver station and ftc dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();

        double testVelocity = Flywheel.shotVelocity; //starts at the shared speed, dpad changes it for this run only
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;
        boolean lastSquare = false;
        ElapsedTime outtakeTimer = new ElapsedTime();
        boolean outtaking = false;

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
            boolean feeding = transfer.feed(gamepad1.right_bumper, flywheel.returnVel(), testVelocity);
            intake.runIntake(gamepad1.left_bumper ? 1 : 0);

            //square outtakes the intake for 0.2 seconds, same if tapped or held
            if (gamepad1.square && !lastSquare) {
                outtaking = true;
                outtakeTimer.reset();
            }
            lastSquare = gamepad1.square;
            if (outtaking && outtakeTimer.seconds() < 0.08) {
                intake.runIntake(-1);
            } else {
                outtaking = false;
            }

            telemetry.addData("Target", testVelocity);
            telemetry.addData("Actual", flywheel.returnVel());
//            telemetry.addData("percentError", percentError);
//            telemetry.addData("error", error);
//            telemetry.addData("Flywheel power", flywheel.returnPower());
//            telemetry.addData("Battery voltage", battery.getVoltage());
//            telemetry.addData("loopMs", loopTimer.milliseconds());
//            telemetry.addData("shooting", gamepad1.right_bumper ? 1 : 0);
//            telemetry.addData("feeding", feeding ? 1 : 0);
            loopTimer.reset();
            telemetry.update();
        }
    }
}