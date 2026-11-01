package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Drivetrain;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Turret;

@Config
@TeleOp
public class Version1 extends LinearOpMode {
    //edit live in the dashboard config tab, flywheel pidf is under Flywheel
    public static double targetVelocity = 1800; //shot speed in ticks per second

    private Flywheel flywheel;
    private Intake intake;
    private Turret turret;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        //sends telemetry to both driver station and ftc dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        drivetrain = new Drivetrain(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        double speed = 1; //flywheel speed

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

            flywheel.update(speed * targetVelocity, battery.getVoltage());

            turret.spinTurret(gamepad2.left_stick_x);

            //Manual control for intake and transfer
            if (gamepad2.right_bumper) {
                intake.runTransfer(1);
            } else {
                intake.runTransfer(0);
            }

            if (gamepad1.right_bumper) {
                intake.flowerDown();
            } else {
                intake.flowerUp();
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
            telemetry.addData("Target velocity", speed * targetVelocity);
            telemetry.addData("Flywheel power", flywheel.returnPower());
            telemetry.addData("Battery voltage", battery.getVoltage());
            //shows what the robot is actually using, to check dashboard edits
            telemetry.addData("kV", Flywheel.kV);
            telemetry.addData("kP", Flywheel.kP);
            telemetry.addData("kD", Flywheel.kD);
            telemetry.update();
        }
    }
}
