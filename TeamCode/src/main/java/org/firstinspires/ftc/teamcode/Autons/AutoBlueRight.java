package org.firstinspires.ftc.teamcode.Autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Intake;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Transfer;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Turret;

@Config
@Autonomous
//same as AutoRedRight mirrored to the opposite corner, every x and y flipped and every angle turned 180
public class AutoBlueRight extends LinearOpMode {
    //flywheel speed from 0 to 1, 1 is fullSpeedVelocity
    public static double flywheelSpeed = 1; //1 is fullSpeedVelocity
    public static double fullSpeedVelocity = 730; //ticks per second at speed 1
    public static double pickupDistance = 13; //inches it drives forward into the flower after the ramp drops
    public static double firstShotMaxWait = 1; //most seconds it waits for the flywheel before the first shot
    public static double readyTolerance = 20; //ticks per second off the target that counts as up to speed
    public static double readyHoldTime = 0.15; //seconds it has to stay up to speed, so passing through the target on the way up doesn't count
    public static double cornerWait = 0.25; //seconds it sits in the corner
    public static double grabTime = 0.9; //seconds it sits in the first flower before backing out and raising the ramp
    public static double farGrabTime = 0.9; //same for the second flower
    public static double tagConfirmTime = 0.1; //seconds a tag has to stay in view to count, stops quick false sightings
    public static double scanTime = 0.25; //seconds to look for a tag on the hive before giving up
    public static double shootTime = 1.1; //seconds the transfer runs to shoot
    public static double farPickupDistance = 13; //inches it moves into the far flower
    public static double crossSpeed = 0.85; //speed multiplier for crossing sides, 0.85 leaves the motors room to correct so it stays smooth

    private Flywheel flywheel;
    private Turret turret;
    private MecanumDrive drive;
    private Intake intakeBoard;
    private Transfer transferBoard;
    private boolean intake = false; //true spins the intake
    private Pose2d farFlower;
    private Pose2d afterFlowerShot; //where it shoots after the second flower and the corner, same as farShot
    private boolean transfer = false; //true spins the transfer
    private VoltageSensor battery;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() {
        //aim at the blue goal, put back to red when the auton ends so teleop isn't affected
        double redGoalY = Turret.goalY;
        Turret.goalY = Math.abs(redGoalY);
        try {
            runAuton();
        } finally {
            Turret.goalY = redGoalY;
        }
    }

    private void runAuton() {
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        turret.resetAngle(); //turret must face robot forward at init
        intakeBoard = new Intake(hardwareMap);
        transferBoard = new Transfer(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();
        dashboard = FtcDashboard.getInstance();

        Pose2d startPose = new Pose2d(-58, 12, Math.toRadians(0));
        drive = new MecanumDrive(hardwareMap, startPose);

        //each path starts where the last one ends
        Pose2d pose0 = new Pose2d(-52, 12, Math.toRadians(0)); //6 inches forward from the start
        Pose2d pose1 = new Pose2d(-24, 58, Math.toRadians(90)); //at the flower
        Pose2d facingFlower = new Pose2d(-24, 58 - pickupDistance, Math.toRadians(90)); //stops short, facing the flower
        Pose2d scanPose = pose0; //scans and shoots again where it shot the first time
        farFlower = new Pose2d(48, 30, Math.toRadians(0)); //second flower, stays facing 0 so it doesn't turn
        Pose2d farShot = new Pose2d(farFlower.position.x + 10, 18, Math.toRadians(180)); //where the cross ends when it shoots, facing 180
        afterFlowerShot = farShot; //shoots from the same spot after the flower and the corner
        Vector2d midway = new Vector2d(20, 10); //first strafe of the cross

        Action path0 = drive.actionBuilder(startPose)
                .strafeTo(pose0.position)
                .build();
        Action path1 = drive.actionBuilder(pose0)
                .splineToLinearHeading(facingFlower, Math.toRadians(70))
                .build();
        Action pickup = drive.actionBuilder(facingFlower)
                .strafeTo(pose1.position)
                .build();
        //ramp goes up right away and it goes straight back to the scan spot, turning on the way
        Action backToHive = drive.actionBuilder(pose1)
                .afterTime(0, () -> intakeBoard.rampUp())
                .strafeToLinearHeading(scanPose.position, scanPose.heading)
                .build();
        //crossing sides goes crossSpeed times faster than the other paths
        VelConstraint crossVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(MecanumDrive.PARAMS.maxWheelVel * crossSpeed),
                new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * crossSpeed)));
        AccelConstraint crossAccel = new ProfileAccelConstraint(
                MecanumDrive.PARAMS.minProfileAccel * crossSpeed, MecanumDrive.PARAMS.maxProfileAccel * crossSpeed);

        //hive standing: back to the flower spot, then the far flower, ramp drops and it strafes left to pick up
        Action toFarFlower = drive.actionBuilder(scanPose) //both moves in one, no stop between
                .strafeToLinearHeading(midway, Math.toRadians(0), crossVel, crossAccel)
                //second strafe as a spline so it rounds the corner instead of stopping
                .splineToLinearHeading(farFlower, arcEndTangent(scanPose.position, midway, farFlower.position), crossVel, crossAccel)
                .build();
        //hive tipped: same cross, ends at the shot spot instead
        Action toFarShot = drive.actionBuilder(scanPose) //both moves in one, no stop between
                .strafeToLinearHeading(midway, Math.toRadians(0), crossVel, crossAccel)
                //second strafe as a spline so it rounds the corner instead of stopping
                .splineToSplineHeading(farShot, arcEndTangent(scanPose.position, midway, farShot.position), crossVel, crossAccel) //turn eases in and out
                .build();

        waitForStart();
        if (isStopRequested()) return;

        run(path0);
        waitForFlywheel(firstShotMaxWait); //shoots as soon as the flywheel is up to speed
        intake = true; //intake and transfer start after the 6 inches
        transfer = true;
        run(new SleepAction(shootTime));
        transfer = false; //done shooting, intake keeps going
        run(path1); //stops facing the flower
        intakeBoard.rampDown(); //ramp drops while it drives in, no wait
        run(pickup); //drives forward into the flower
        run(new SleepAction(grabTime)); //a little longer to grab the balls
        run(backToHive); //back to where it shot the first time
        boolean hiveStanding = scanForTag(); //sees a tag only if the hive hasn't tipped
        if (hiveStanding) { //shoot again
            transfer = true;
            run(new SleepAction(shootTime));
            transfer = false;
        }

        if (hiveStanding) {
            intake = false; //intake off while crossing sides
            run(toFarFlower);
            intake = true; //movement done, intake back on
            farFlowerAndShoot(); //picks up, turns around on the way to the shot spot and shoots

            cornerPickup(afterFlowerShot, null);
            shoot();
        } else {
            intake = false; //intake stops right before crossing sides
            run(toFarShot);
            intake = true; //movement done, intake back on
            run(new SleepAction(0.5)); //lets the flywheel settle before shooting
            shoot(); //shoots the balls from the first flower

            //turns around on the way to the second flower, then the same flower and corner steps as the other case
            run(drive.actionBuilder(farShot)
                    .strafeToLinearHeading(farFlower.position, farFlower.heading)
                    .build());
            farFlowerAndShoot(); //picks up, turns around on the way to the shot spot and shoots
            cornerPickup(afterFlowerShot, null); //ends at the shot spot facing 180

            shoot(); //shoots on this side either way
        }

        flywheel.update(0, battery.getVoltage());
        intakeBoard.runIntake(0);
        transferBoard.run(0);
    }

    //corner pickup, starts and ends at the shot spot facing 180, intake stays on
    //backOut is a spot to back up to first, null goes straight there
    private void cornerPickup(Pose2d start, Vector2d backOut) {
        Vector2d cornerFront = new Vector2d(39, 60);
        Vector2d corner = new Vector2d(39, 72); //pushes into the wall
        TrajectoryActionBuilder toCorner = drive.actionBuilder(start);
        if (backOut != null) {
            toCorner = toCorner.strafeTo(backOut);
        }
        run(toCorner
                .strafeToLinearHeading(cornerFront, Math.toRadians(90))
                .strafeTo(corner)
                .build());
        run(new SleepAction(cornerWait));
        //goes back exactly the way it came
        run(drive.actionBuilder(new Pose2d(corner, Math.toRadians(90)))
                .strafeTo(cornerFront)
                .strafeToLinearHeading(afterFlowerShot.position, afterFlowerShot.heading)
                .build());
    }

    //at the far flower facing 0: ramp drops, drives in, grabs, backs out, turns around on the way to the shot spot and shoots
    private void farFlowerAndShoot() {
        Vector2d pickupSpot = new Vector2d(farFlower.position.x + farPickupDistance, farFlower.position.y);
        intakeBoard.rampDown(); //ramp drops while it drives in, no wait
        run(drive.actionBuilder(farFlower)
                .strafeTo(pickupSpot) //forward into the flower
                .build());
        run(new SleepAction(farGrabTime)); //a little longer to grab the balls
        intakeBoard.rampUp(); //ramp goes up as it starts backing out, so it's up before the turn
        run(drive.actionBuilder(new Pose2d(pickupSpot, farFlower.heading))
                .strafeTo(farFlower.position) //backs out
                .strafeToLinearHeading(afterFlowerShot.position, afterFlowerShot.heading) //turns around on the way to the shot spot
                .build());
        shoot();
    }

    //runs the transfer for shootTime
    private void shoot() {
        transfer = true;
        run(new SleepAction(shootTime));
        transfer = false;
    }

    //end direction that makes the spline after a strafe one smooth curve instead of an s-bend
    private static double arcEndTangent(Vector2d start, Vector2d corner, Vector2d end) {
        double chord = directionTo(corner, end);
        double into = directionTo(start, corner);
        double bend = Math.atan2(Math.sin(chord - into), Math.cos(chord - into));
        return chord + bend;
    }

    //direction in radians pointing from one spot to another
    private static double directionTo(Vector2d from, Vector2d to) {
        return Math.atan2(to.y - from.y, to.x - from.x);
    }

    //waits until the flywheel stays up to speed for readyHoldTime, or maxWait runs out
    private void waitForFlywheel(double maxWait) {
        ElapsedTime waitTimer = new ElapsedTime();
        ElapsedTime readyTimer = new ElapsedTime();
        boolean ready = false;
        while (opModeIsActive() && waitTimer.seconds() < maxWait) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            double target = flywheelSpeed * fullSpeedVelocity;
            flywheel.update(target, battery.getVoltage());
            turret.aimAtGoal(pose, new PoseVelocity2d(new Vector2d(0, 0), 0));
            intakeBoard.runIntake(intake ? 1 : 0);
            transferBoard.run(transfer ? 1 : 0);
            if (Math.abs(target - flywheel.returnVel()) <= readyTolerance) {
                if (!ready) {
                    ready = true;
                    readyTimer.reset();
                }
                if (readyTimer.seconds() >= readyHoldTime) {
                    return;
                }
            } else {
                ready = false;
            }
        }
    }

    //waits up to scanTime for a tag on the hive, true if it sees one
    private boolean scanForTag() {
        ElapsedTime scanTimer = new ElapsedTime();
        ElapsedTime seenTimer = new ElapsedTime();
        boolean seeing = false;
        while (opModeIsActive() && scanTimer.seconds() < scanTime) {
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();
            flywheel.update(flywheelSpeed * fullSpeedVelocity, battery.getVoltage());
            turret.aimAtGoal(pose, new PoseVelocity2d(new Vector2d(0, 0), 0));
            intakeBoard.runIntake(intake ? 1 : 0);
            transferBoard.run(transfer ? 1 : 0);
            //the tag has to stay in view the whole tagConfirmTime, one flicker doesn't count
            if (turret.seesGoalTag(pose)) {
                if (!seeing) {
                    seeing = true;
                    seenTimer.reset();
                }
                if (seenTimer.seconds() >= tagConfirmTime) {
                    return true;
                }
            } else {
                seeing = false;
            }
        }
        return false;
    }

    //runs one action until it's done, keeps the flywheel updated like version 1
    private void run(Action action) {
        boolean running = true;
        while (opModeIsActive() && running) {
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);

            double flywheelTarget = flywheelSpeed * fullSpeedVelocity;
            flywheel.update(flywheelTarget, battery.getVoltage());
            Pose2d pose = drive.localizer.getPose();
            turret.aimAtGoal(pose, new PoseVelocity2d(new Vector2d(0, 0), 0));
            intakeBoard.runIntake(intake ? 1 : 0);
            transferBoard.run(transfer ? 1 : 0);

            packet.put("Velocity", flywheel.returnVel());
            packet.put("targetVelocity", flywheelTarget);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
