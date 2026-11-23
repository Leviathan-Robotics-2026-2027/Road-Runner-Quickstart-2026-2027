package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Drivetrain;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Turret;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Transfer;

//same as version 1, gamepad 1 drives and runs the ramp, gamepad 2 does everything else
//uses the version 1 settings in the dashboard
@TeleOp(name = "TeleRedLeft")
public class TeleRedLeft extends LinearOpMode {
    private Flywheel flywheel;
    private Intake intake;
    private Transfer transfer;
    private Turret turret;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        //sends telemetry to both driver station and ftc dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-60, -60, Math.toRadians(0)));
        drivetrain = new Drivetrain(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap); //ramp starts straight up
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        double speed = 1; //flywheel speed
        turret.resetAngle(); //turret must face robot forward at init
        boolean lastRampButton = false;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastSquare = false;
        boolean lastCircle = false;
        boolean outtaking = false;
        ElapsedTime outtakeTimer = new ElapsedTime();

        waitForStart();
        boolean findingStart = turret.canFindStart();

        while (!isStopRequested() && opModeIsActive()) {
            PoseVelocity2d robotVel = drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            //square resets the location to the corner on the side the robot is on, heading stays from odometry
            if (gamepad2.square && !lastSquare) {
                double cornerX = currentPose.position.x > Turret.sideLineX ? 60 : -60;
                currentPose = new Pose2d(cornerX, -60, currentPose.heading.toDouble());
                drive.localizer.setPose(currentPose);
                gamepad1.rumble(300); //tells the driver the reset happened
            }
            lastSquare = gamepad2.square;

            //gamepad 1 drives
            boolean sticksReleased = Math.abs(gamepad1.left_stick_y) < 0.05
                    && Math.abs(gamepad1.left_stick_x) < 0.05
                    && Math.abs(gamepad1.right_stick_x) < 0.05;
            double driveSpeed = TeleRedLeft_SingleController.driveSpeed;
            if (sticksReleased) {
                drivetrain.brakeMotors();
            } else {
                drivetrain.driveMotors(gamepad1.left_stick_y * driveSpeed, gamepad1.right_stick_x * driveSpeed, gamepad1.left_stick_x * driveSpeed);
            }
            double distanceToGoal = Turret.getDistanceToGoal(currentPose);
            //no distance speed during the sweep, the location isn't known yet
            double shotVelocity = TeleRedLeft_SingleController.distanceSpeed && !findingStart ? TeleRedLeft_SingleController.velocityForDistance(distanceToGoal) : TeleRedLeft_SingleController.targetVelocity;
            flywheel.update(speed * shotVelocity, battery.getVoltage());

            //gamepad 1 right bumper toggles the ramp up and down
            if (gamepad1.right_bumper && !lastRampButton) {
                intake.toggleRamp();
            }
            lastRampButton = gamepad1.right_bumper;

            //right trigger is like the old stick pushed right, left trigger like pushed left
            double manualTurret = gamepad2.right_trigger - gamepad2.left_trigger;

            //dpad nudges the auto aim left or right, one step per press
            if (gamepad2.dpad_left && !lastDpadLeft) {
                turret.adjustAim(1);
            }
            if (gamepad2.dpad_right && !lastDpadRight) {
                turret.adjustAim(-1);
            }
            lastDpadLeft = gamepad2.dpad_left;
            lastDpadRight = gamepad2.dpad_right;

            //any stick or trigger cancels the startup sweep
            if (findingStart && (!sticksReleased || Math.abs(manualTurret) > 0.1)) {
                findingStart = false;
            }

            if (findingStart) {
                Pose2d startPose = turret.findStartPose();
                if (startPose != null) {
                    drive.localizer.setPose(startPose);
                    findingStart = false;
                }
            } else if (Math.abs(manualTurret) > 0.1) {
                turret.spinTurret(manualTurret);
            } else if (gamepad2.triangle && turret.aimAtTag(currentPose)) {
                //holding triangle points the turret straight at the april tags
            } else {
                turret.aimAtGoal(currentPose, robotVel);
            }

            //gamepad 2 runs the intake and transfer
            //only feeds while the flywheel is up to speed, see Transfer speedGate
            boolean feeding = transfer.feed(gamepad2.right_bumper, flywheel.returnVel(), speed * shotVelocity);

            intake.runIntake(gamepad2.left_bumper ? 1 : 0);


            //circle outtakes the intake for 0.08 seconds to spit out one ball, same if tapped or held
            if (gamepad2.circle && !lastCircle) {
                outtaking = true;
                outtakeTimer.reset();
            }
            lastCircle = gamepad2.circle;
            if (outtaking && outtakeTimer.seconds() < 0.08) {
                intake.runIntake(-1);
            } else {
                outtaking = false;
            }

            telemetry.addData("velocity", flywheel.returnVel());
            telemetry.addData("targetVelocity", speed * shotVelocity);
            telemetry.addData("feeding", feeding ? 1 : 0);
            telemetry.update();
        }
    }
}