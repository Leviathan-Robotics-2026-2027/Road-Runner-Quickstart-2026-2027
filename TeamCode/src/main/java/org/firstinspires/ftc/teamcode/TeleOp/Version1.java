package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Drivetrain;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Turret;

@TeleOp
public class Version1 extends LinearOpMode {
    private Flywheel flywheel;
    private Intake intake;
    private Turret turret;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        drivetrain = new Drivetrain(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        flywheel.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double speed = 1;

        PIDFCoefficients pidf = new PIDFCoefficients(30, 0, 0, 19.5);
        flywheel.flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            boolean sticksReleased = Math.abs(gamepad1.left_stick_y) < 0.05
                    && Math.abs(gamepad1.left_stick_x) < 0.05
                    && Math.abs(gamepad1.right_stick_x) < 0.05;
            if (sticksReleased) {
                drivetrain.brakeMotors();
            } else {
                drivetrain.driveMotors(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
            }
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            flywheel.flywheel.setVelocity(speed * 1800);

            turret.spinTurret(gamepad2.left_stick_x);

            //Manual control for intake and transfer
            if (gamepad2.right_bumper) {
                intake.runTransfer(1);
            } else {
                intake.runTransfer(0);
            }

            if (gamepad2.left_bumper) {
                intake.runIntake(1);
            } else {
                intake.runIntake(0);
            }

            //Outake system
            if (gamepad2.square) {
                intake.runIntake(-1);
            }
            if (gamepad2.circle) {
                intake.runTransfer(-1);
            }

            telemetry.addData("Velocity", flywheel.returnVel());
            telemetry.update();
        }
    }
}
