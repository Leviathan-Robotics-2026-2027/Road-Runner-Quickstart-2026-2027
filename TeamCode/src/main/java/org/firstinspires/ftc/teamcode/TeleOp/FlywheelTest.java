package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;

@TeleOp
public class FlywheelTest extends LinearOpMode {
    private Flywheel flywheel;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new Flywheel(hardwareMap);

        flywheel.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double speed = 1;

        PIDFCoefficients pidf = new PIDFCoefficients(150, 0, 0, 10);
        flywheel.flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (gamepad1.right_bumper) {
                flywheel.flywheel.setVelocity(speed * 6000);
            }
        }
    }
}
