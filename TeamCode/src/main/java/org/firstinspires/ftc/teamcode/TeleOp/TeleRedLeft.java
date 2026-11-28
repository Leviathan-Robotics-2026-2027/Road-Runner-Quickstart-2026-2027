package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Turret;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Transfer;

//same as version 1, gamepad 1 drives and runs the ramp, gamepad 2 does everything else
//uses the version 1 settings in the dashboard
@TeleOp(name = "TeleRedLeft")
public class TeleRedLeft extends LinearOpMode {
    //gamepad 1 triangle: drives to the shot spot, waits for gamepad 2 to shoot, then gives driving back
    private static final int DRIVING = 0; //gamepad 1 drives normally
    private static final int GOING = 1; //driving itself to the shot spot
    private static final int WAITING = 2; //at the shot spot, waiting for gamepad 2 to hold right bumper
    private static final int SHOOTING = 3; //gamepad 2 is holding right bumper
    private static final double shotSpeed = 2.25; //speed multiplier for driving to the shot spot
    private static final double freezeSpeed = 18; //inches per second while gamepad 2 holds dpad up or down during the freeze, 0.5 sec is 9 inches
    private Flywheel flywheel;
    private Intake intake;
    private Transfer transfer;
    private Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {

        //sends telemetry to both driver station and ftc dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-60, -60, Math.toRadians(0)));
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
        boolean lastCircle = false;
        boolean lastSquare = false;
        boolean outtaking = false;
        boolean lastTriangle = false;
        int shotMode = DRIVING;
        Action goToShot = null;
        Pose2d shotTarget = null; //where triangle is driving to, facing the goal
        //goes shotSpeed times faster than the normal road runner limits
        VelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(MecanumDrive.PARAMS.maxWheelVel * shotSpeed),
                new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * shotSpeed)));
        AccelConstraint fastAccel = new ProfileAccelConstraint(
                MecanumDrive.PARAMS.minProfileAccel * shotSpeed, MecanumDrive.PARAMS.maxProfileAccel * shotSpeed);
        ElapsedTime outtakeTimer = new ElapsedTime();

        waitForStart();
        ElapsedTime matchTimer = new ElapsedTime(); //teleop is 2 minutes
        boolean warned30 = false;
        boolean warned10 = false;
        boolean warnedEnd = false;
        boolean findingStart = turret.canFindStart();

        while (!isStopRequested() && opModeIsActive()) {
            //3 rumbles with 30 seconds left, 2 rumbles with 10 seconds left
            if (!warned30 && matchTimer.seconds() >= 90) {
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
                warned30 = true;
            }
            if (!warned10 && matchTimer.seconds() >= 110) {
                gamepad1.rumbleBlips(2);
                gamepad2.rumbleBlips(2);
                warned10 = true;
            }
            //teleop is over at 2 minutes, rumbles until it's stopped
            if (!warnedEnd && matchTimer.seconds() >= 120) {
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                gamepad2.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                warnedEnd = true;
            }

            PoseVelocity2d robotVel = drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            //circle resets the location to the corner on the side the robot is on, heading stays from odometry
            if (gamepad2.circle && !lastCircle) {
                double cornerX = currentPose.position.x > Turret.sideLineX ? 60 : -60;
                currentPose = new Pose2d(cornerX, -60, currentPose.heading.toDouble());
                drive.localizer.setPose(currentPose);
                gamepad1.rumble(1000); //1 second rumble on both so everyone knows the reset happened
                gamepad2.rumble(1000);
            }
            lastCircle = gamepad2.circle;

            //gamepad 1 triangle drives to the shot spot on this side, press again to cancel
            if (gamepad1.triangle && !lastTriangle) {
                if (shotMode == DRIVING) {
                    Vector2d shotSpot = new Vector2d(currentPose.position.x > Turret.sideLineX ? 50 : -50, -11);
                    //turns on the way so it ends up facing the goal on that side
                    double faceGoal = Math.atan2(Turret.goalY - shotSpot.y, Turret.getGoalX(new Pose2d(shotSpot, 0)) - shotSpot.x);
                    shotTarget = new Pose2d(shotSpot, faceGoal);
                    goToShot = drive.actionBuilder(currentPose)
                            .strafeToLinearHeading(shotTarget.position, shotTarget.heading, fastVel, fastAccel)
                            .build();
                    shotMode = GOING;
                } else if (shotMode == GOING || shotMode == WAITING) {
                    shotMode = DRIVING; //cancelled
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                }
            }
            lastTriangle = gamepad1.triangle;

            boolean sticksReleased = Math.abs(gamepad1.left_stick_y) < 0.05
                    && Math.abs(gamepad1.left_stick_x) < 0.05
                    && Math.abs(gamepad1.right_stick_x) < 0.05;
            if (shotMode == GOING) {
                if (!goToShot.run(new TelemetryPacket())) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    shotMode = WAITING;
                    gamepad2.rumble(500); //tells gamepad 2 to hold right bumper and shoot
                }
            } else if (shotMode == WAITING) {
                drive.setDrivePowers(freezeDrive(battery.getVoltage())); //only gamepad 2 dpad up and down move it
                if (gamepad2.right_bumper) {
                    shotMode = SHOOTING;
                }
            } else if (shotMode == SHOOTING) {
                drive.setDrivePowers(freezeDrive(battery.getVoltage())); //gamepad 1 can't drive while gamepad 2 shoots, dpad up and down still move it
                if (!gamepad2.right_bumper) {
                    shotMode = DRIVING;
                    gamepad1.rumble(300); //driving is back
                }
            } else {
                //gamepad 1 drives
                //same driving as localization test, full speed, sticks released stops and brakes
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                        -gamepad1.right_stick_x * 0.9)); //turning at 0.9x so it is a little easier to control
            }
            double distanceToGoal = Turret.getDistanceToGoal(currentPose);
            //no distance speed during the sweep, the location isn't known yet
            double shotVelocity = TeleRedLeft_SingleController.distanceSpeed && !findingStart ? TeleRedLeft_SingleController.velocityForDistance(distanceToGoal) : Flywheel.shotVelocity;
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


            //square outtakes the intake for 0.08 seconds to spit out one ball, same if tapped or held
            if (gamepad2.square && !lastSquare) {
                outtaking = true;
                outtakeTimer.reset();
            }
            lastSquare = gamepad2.square;
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

    //during the freeze, holding gamepad 2 dpad up drives forward and dpad down drives back, nothing held stays still
    //uses the drive's tuned kS and kV to turn freezeSpeed into motor power
    private PoseVelocity2d freezeDrive(double batteryVoltage) {
        double direction = gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0;
        if (direction == 0) {
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }
        double volts = MecanumDrive.PARAMS.kS + MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick * freezeSpeed;
        double power = Math.min(1, volts / batteryVoltage);
        return new PoseVelocity2d(new Vector2d(direction * power, 0), 0);
    }
}
