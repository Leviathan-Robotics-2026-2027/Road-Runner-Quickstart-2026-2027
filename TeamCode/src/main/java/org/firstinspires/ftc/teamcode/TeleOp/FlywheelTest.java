package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;

@TeleOp
public class FlywheelTest extends LinearOpMode {
    private Flywheel flywheel;
    private Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);

        flywheel.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double speed = 1;

        PIDFCoefficients pidf = new PIDFCoefficients(200, 0, 0, 15);
        flywheel.flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            flywheel.flywheel.setVelocity(speed * 1600);

            intake.runTransfer(gamepad1.right_bumper ? 1 : 0);
            intake.runIntake(gamepad1.left_bumper ? 1 : 0);

            telemetry.addData("velocity", flywheel.returnVel());
            telemetry.update();
        }
    }
}
