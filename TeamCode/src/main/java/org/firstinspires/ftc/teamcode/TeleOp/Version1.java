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

@Config
@TeleOp
public class Version1 extends LinearOpMode {
    //edit live in the dashboard config tab, flywheel pidf is under Flywheel
    public static double targetVelocity = 1400; //close shot speed in ticks per second
    public static double driveSpeed = 0.4; //drivetrain speed, 1 is full

    //flywheel speeds up the farther the robot is from the goal
    public static boolean distanceSpeed = true; //false uses targetVelocity everywhere
    public static double closeDistance = 30; //inches, at or closer than this uses targetVelocity
    public static double farDistance = 64; //inches, start pose (-60,-60) to the goal, the farthest shot
    public static double farVelocity = 1550; //shot speed at farDistance CHANGE CHANGE CHANGE CHANGE CHANGE
    public static double maxVelocity = 1550; //speed never goes above this

    private Flywheel flywheel;
    private Intake intake;
    private Turret turret;
    private Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        //sends telemetry to both driver station and ftc dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-60, -60, Math.toRadians(0)));
        drivetrain = new Drivetrain(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);

        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        double speed = 1; //flywheel speed
        turret.resetAngle(); //turret must face robot forward at init
        boolean autoAim = true;
        boolean lastTriangle = false;

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            PoseVelocity2d robotVel = drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            boolean sticksReleased = Math.abs(gamepad1.left_stick_y) < 0.05
                    && Math.abs(gamepad1.left_stick_x) < 0.05
                    && Math.abs(gamepad1.right_stick_x) < 0.05;
            if (sticksReleased) {
                drivetrain.brakeMotors();
            } else {
                drivetrain.driveMotors(gamepad1.left_stick_y * driveSpeed, gamepad1.right_stick_x * driveSpeed, gamepad1.left_stick_x * driveSpeed);
            }
            double distanceToGoal = Turret.getDistanceToGoal(currentPose);
            double shotVelocity = distanceSpeed ? velocityForDistance(distanceToGoal) : targetVelocity;
            flywheel.update(speed * shotVelocity, battery.getVoltage());

            //triangle toggles auto aim, moving the stick takes over manually
            if (gamepad2.triangle && !lastTriangle) {
                autoAim = !autoAim;
            }
            lastTriangle = gamepad2.triangle;

            if (Math.abs(gamepad2.left_stick_x) > 0.1) {
                turret.spinTurret(gamepad2.left_stick_x);
            } else if (autoAim) {
                turret.aimAtGoal(currentPose, robotVel);
            } else {
                turret.spinTurret(0);
            }

            //Manual control for intake and transfer
            if (gamepad1.right_bumper) {
                intake.runTransfer(1);
            } else {
                intake.runTransfer(0);
            }

//            if (gamepad1.right_bumper) {
//                intake.flowerDown();
//            } else {
//                intake.flowerUp();
//            }

            if (gamepad1.left_bumper) {
                intake.runIntake(1);
            } else {
                intake.runIntake(0);
            }

            //Outake system
            if (gamepad1.square) {
                intake.runIntake(-1);
            }
            if (gamepad1.circle) {
                intake.runTransfer(-1);
            }

            telemetry.addData("Velocity", flywheel.returnVel());
            telemetry.addData("targetVelocity", speed * shotVelocity);
            telemetry.update();
        }
    }

    //straight line through the close and far speeds, kept between the close speed and maxVelocity
    private double velocityForDistance(double distance) {
        double slope = (farVelocity - targetVelocity) / (farDistance - closeDistance);
        double velocity = targetVelocity + slope * (distance - closeDistance);
        return Math.max(targetVelocity, Math.min(maxVelocity, velocity));
    }
}
