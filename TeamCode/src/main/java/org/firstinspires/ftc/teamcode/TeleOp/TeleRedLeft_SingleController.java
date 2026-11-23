package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Drivetrain;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Turret;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Transfer;

@Config
@TeleOp(name = "TeleRedLeft_SingleController")
public class TeleRedLeft_SingleController extends LinearOpMode {
    //edit live in the dashboard config tab, flywheel pidf is under Flywheel
    public static double targetVelocity = 740; //close shot speed in ticks per second
    public static double driveSpeed = 0.7; //drivetrain speed, 1 is full

    //flywheel speeds up the farther the robot is from the goal
    public static boolean distanceSpeed = false; //false uses targetVelocity everywhere
    public static double closeDistance = 32; //inches, at or closer than this uses targetVelocity
    public static double farDistance = 48; //inches of the farthest shot distance
    public static double farVelocity = 760; //shot speed at farDistance
    public static double maxVelocity = 760; //speed never goes above this

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
        boolean lastCross = false;
        boolean lastDpadLeft = false;
        boolean lastDpadRight = false;
        boolean lastSquare = false;

        waitForStart();
        boolean findingStart = turret.canFindStart();

        while (!isStopRequested() && opModeIsActive()) {
            PoseVelocity2d robotVel = drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            //square resets the location to the start, heading stays from odometry
            if (gamepad1.square && !lastSquare) {
                currentPose = new Pose2d(-60, -60, currentPose.heading.toDouble());
                drive.localizer.setPose(currentPose);
                gamepad1.rumble(300); //tells the driver the reset happened
            }
            lastSquare = gamepad1.square;

            boolean sticksReleased = Math.abs(gamepad1.left_stick_y) < 0.05
                    && Math.abs(gamepad1.left_stick_x) < 0.05
                    && Math.abs(gamepad1.right_stick_x) < 0.05;
            if (sticksReleased) {
                drivetrain.brakeMotors();
            } else {
                drivetrain.driveMotors(gamepad1.left_stick_y * driveSpeed, gamepad1.right_stick_x * driveSpeed, gamepad1.left_stick_x * driveSpeed);
            }
            double distanceToGoal = Turret.getDistanceToGoal(currentPose);
            //no distance speed during the sweep, the location isn't known yet
            double shotVelocity = distanceSpeed && !findingStart ? velocityForDistance(distanceToGoal) : targetVelocity;
            flywheel.update(speed * shotVelocity, battery.getVoltage());

            //cross toggles the ramp up and down
            if (gamepad1.cross && !lastCross) {
                intake.toggleRamp();
            }
            lastCross = gamepad1.cross;

            //right trigger is like the old stick pushed right, left trigger like pushed left
            double manualTurret = gamepad1.right_trigger - gamepad1.left_trigger;

            //dpad nudges the auto aim left or right, one step per press
            if (gamepad1.dpad_left && !lastDpadLeft) {
                turret.adjustAim(1);
            }
            if (gamepad1.dpad_right && !lastDpadRight) {
                turret.adjustAim(-1);
            }
            lastDpadLeft = gamepad1.dpad_left;
            lastDpadRight = gamepad1.dpad_right;

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
            } else if (gamepad1.triangle && turret.aimAtTag(currentPose)) {
                //holding triangle points the turret straight at the april tags
            } else {
                turret.aimAtGoal(currentPose, robotVel);
            }

            //Manual control for intake and transfer
            //only feeds while the flywheel is up to speed, see Transfer speedGate
            boolean feeding = transfer.feed(gamepad1.right_bumper, flywheel.returnVel(), speed * shotVelocity);

//            if (gamepad1.right_bumper) {
//                intake.rampDown();
//            } else {
//                intake.rampUp();
//            }

            intake.runIntake(gamepad1.left_bumper ? 1 : 0);

            //circle runs the intake and transfer backwards
            if (gamepad1.circle) {
                intake.runIntake(-1);
                transfer.run(-1);
            }

            telemetry.addData("velocity", flywheel.returnVel());
            telemetry.addData("targetVelocity", speed * shotVelocity);
            telemetry.addData("feeding", feeding ? 1 : 0);
            telemetry.update();
        }
    }

    //straight line through the close and far speeds, kept between the close speed and maxVelocity
    static double velocityForDistance(double distance) {
        double slope = (farVelocity - targetVelocity) / (farDistance - closeDistance);
        double velocity = targetVelocity + slope * (distance - closeDistance);
        return Math.max(targetVelocity, Math.min(maxVelocity, velocity));
    }
}