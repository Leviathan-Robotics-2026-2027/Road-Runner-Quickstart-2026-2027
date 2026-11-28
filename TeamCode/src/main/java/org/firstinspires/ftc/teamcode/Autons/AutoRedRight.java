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
import com.acmerobotics.roadrunner.SequentialAction;
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
public class AutoRedRight extends LinearOpMode {
    public static double pickupDistance = 15; //inches it drives forward into the flower after the ramp drops
    public static double firstShotMaxWait = 1; //most seconds it waits for the flywheel before the first shot
    public static double readyTolerance = 20; //ticks per second off the target that counts as up to speed
    public static double readyHoldTime = 0.15; //seconds it has to stay up to speed, so passing through the target on the way up doesn't count
    public static double cornerWait = 0; //seconds it sits in the corner
    public static double preShootWait = 0.25; //seconds it waits before the transfer turns on for every shot
    public static double farFlowerShotExtraWait = 0.15; //extra wait before the shot after the second flower
    public static double firstRampEarlyTime = 0.5; //seconds before reaching the first flower that the ramp starts dropping
    public static double flowerAimOffset = -4; //degrees the aim moves again before and again after the second flower, negative is right
    public static double crossAimOffset = -4; //degrees the turret aim moves after the cross, negative is right
    public static double rampEarlyTime = 0.75; //same for the second flower
    public static double rampRetractDistance = 6; //inches out of the flower before the ramp goes up, happens while moving so it adds no time
    public static double grabTime = 1.05; //seconds it sits in the first flower before backing out and raising the ramp
    public static double farGrabTime = 1.05; //same for the second flower
    public static double tagConfirmTime = 0.1; //seconds a tag has to stay in view to count, stops quick false sightings
    public static double scanTime = 0.25; //seconds to look for a tag on the hive before giving up
    public static double shootTime = 1.25; //seconds the transfer runs to shoot
    public static double farPickupDistance = 11; //inches it moves into the far flower
    public static double crossSpeed = 0.75; //speed multiplier for crossing sides, lower keeps the wheels from slipping on the curve

    private Flywheel flywheel;
    private Turret turret;
    private MecanumDrive drive;
    private Intake intakeBoard;
    private Transfer transferBoard;
    private boolean intake = false; //true spins the intake
    private Pose2d farFlower;
    private Pose2d afterFlowerShot; //where it shoots after the second flower and the corner, same as farShot
    private boolean transfer = false; //true spins the transfer
    private boolean crossed = false; //true once it's on the other side, shots there re-aim with the limelight
    private boolean tagAim = false; //true points the turret at the april tags when it sees them
    private VoltageSensor battery;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new Flywheel(hardwareMap);
        turret = new Turret(hardwareMap);
        turret.resetAngle(); //turret must face robot forward at init
        intakeBoard = new Intake(hardwareMap);
        transferBoard = new Transfer(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();
        dashboard = FtcDashboard.getInstance();

        Pose2d startPose = new Pose2d(58, -12, Math.toRadians(180)); //same as meep meep
        drive = new MecanumDrive(hardwareMap, startPose);

        //paths from meep meep, each one starts where the last one ends
        Pose2d pose0 = new Pose2d(52, -12, Math.toRadians(180)); //6 inches forward from the start
        Pose2d pose1 = new Pose2d(24, -58, Math.toRadians(-90)); //at the flower
        Pose2d facingFlower = new Pose2d(24, -58 + pickupDistance, Math.toRadians(-90)); //stops short, facing the flower
        Pose2d scanPose = pose0; //scans and shoots again where it shot the first time
        farFlower = new Pose2d(-48, -35, Math.toRadians(180)); //second flower, stays facing 180 so it doesn't turn
        Pose2d farShot = new Pose2d(farFlower.position.x - 10, -18, Math.toRadians(0)); //where the cross ends when it shoots, facing 0
        afterFlowerShot = farShot; //shoots from the same spot after the flower and the corner
        Vector2d midway = new Vector2d(-20, -10); //first strafe of the cross, from meep meep

        Action path0 = drive.actionBuilder(startPose)
                .strafeTo(pose0.position)
                .build();
        Action path1 = drive.actionBuilder(pose0)
                .splineToLinearHeading(facingFlower, Math.toRadians(-110))
                .build();
        Action pickup = drive.actionBuilder(facingFlower)
                .strafeTo(pose1.position)
                .build();
        //goes straight back to the scan spot, turning on the way, ramp goes up once it's out of the flower
        Action backToHive = drive.actionBuilder(pose1)
                .afterDisp(rampRetractDistance, () -> intakeBoard.rampUp())
                .strafeToLinearHeading(scanPose.position, scanPose.heading)
                .build();
        //crossing sides goes crossSpeed times faster than the other paths
        VelConstraint crossVel = new MinVelConstraint(Arrays.asList(
                drive.kinematics.new WheelVelConstraint(MecanumDrive.PARAMS.maxWheelVel * crossSpeed),
                new AngularVelConstraint(MecanumDrive.PARAMS.maxAngVel * crossSpeed)));
        AccelConstraint crossAccel = new ProfileAccelConstraint(
                MecanumDrive.PARAMS.minProfileAccel * crossSpeed, MecanumDrive.PARAMS.maxProfileAccel * crossSpeed);

        //cross for a tipped hive, it starts this right away and scans on the way
        Action toFarShot = drive.actionBuilder(scanPose) //both moves in one, no stop between
                .strafeToLinearHeading(midway, Math.toRadians(180), crossVel, crossAccel)
                //second strafe as a spline so it rounds the corner instead of stopping
                .splineToSplineHeading(farShot, arcEndTangent(scanPose.position, midway, farShot.position), crossVel, crossAccel) //turn eases in and out
                .build();

        waitForStart();
        if (isStopRequested()) return;

        run(path0);
        waitForFlywheel(firstShotMaxWait); //shoots as soon as the flywheel is up to speed
        intake = true; //intake starts after the 6 inches
        shoot(); //done shooting, intake keeps going
        runRampEarly(path1, firstRampEarlyTime); //stops facing the flower, ramp drops on the way
        run(pickup); //drives forward into the flower
        run(new SleepAction(grabTime)); //a little longer to grab the balls
        run(backToHive); //back to where it shot the first time
        //starts crossing right away and scans on the way, only stops to shoot if the hive is still standing
        boolean hiveStanding = runAndScan(toFarShot);
        if (hiveStanding) {
            shoot(); //intake still on
            intake = false; //intake off while crossing sides
            Pose2d here = drive.localizer.getPose();
            runRampEarly(rampEarlyTime, drive.actionBuilder(here)
                    .strafeToLinearHeading(midway, Math.toRadians(180), crossVel, crossAccel)
                    .splineToLinearHeading(farFlower, arcEndTangent(here.position, midway, farFlower.position), crossVel, crossAccel)
                    .build());
            intake = true; //movement done, intake back on
            crossed = true;
            turret.addAimOffset(crossAimOffset); //aim is skewed on this side
            farFlowerAndShoot(); //picks up, turns around on the way to the shot spot and shoots

            cornerPickup(afterFlowerShot, null);
            shoot();
        } else {
            intake = true; //cross is done, intake back on
            crossed = true;
            turret.addAimOffset(crossAimOffset); //aim is skewed on this side
            run(new SleepAction(0.5)); //lets the flywheel settle before shooting
            shoot(); //shoots the balls from the first flower

            //turns around on the way to the second flower, then the same flower and corner steps as the other case
            runRampEarly(rampEarlyTime, drive.actionBuilder(farShot)
                    .strafeToLinearHeading(farFlower.position, farFlower.heading)
                    .build());
            farFlowerAndShoot(); //picks up, turns around on the way to the shot spot and shoots
            cornerPickup(afterFlowerShot, null); //ends at the shot spot facing 0

            shoot(); //shoots on this side either way
        }

        //parks at the end
        run(drive.actionBuilder(afterFlowerShot)
                .strafeTo(new Vector2d(-46, -62))
                .build());

        flywheel.update(0, battery.getVoltage());
        intakeBoard.runIntake(0);
        transferBoard.run(0);
    }

    //corner pickup from meep meep, starts and ends at the shot spot facing 0, intake stays on
    //backOut is a spot to back up to first, null goes straight there
    private void cornerPickup(Pose2d start, Vector2d backOut) {
        Vector2d cornerFront = new Vector2d(-44, -60);
        Vector2d corner = new Vector2d(-44, -68); //pushes into the wall
        TrajectoryActionBuilder toCorner = drive.actionBuilder(start);
        if (backOut != null) {
            toCorner = toCorner.strafeTo(backOut);
        }
        run(toCorner
                .strafeToLinearHeading(cornerFront, Math.toRadians(270))
                .strafeTo(corner)
                .build());
        run(new SleepAction(cornerWait));
        //goes back exactly the way it came
        run(drive.actionBuilder(new Pose2d(corner, Math.toRadians(270)))
                .strafeTo(cornerFront)
                .strafeToLinearHeading(afterFlowerShot.position, afterFlowerShot.heading)
                .build());
    }

    //at the far flower facing 180: ramp drops, drives in, grabs, backs out, turns around on the way to the shot spot and shoots
    private void farFlowerAndShoot() {
        turret.addAimOffset(flowerAimOffset); //before the second flower
        Vector2d pickupSpot = new Vector2d(farFlower.position.x - farPickupDistance, farFlower.position.y);
        run(drive.actionBuilder(farFlower)
                .strafeTo(pickupSpot) //forward into the flower
                .build());
        run(new SleepAction(farGrabTime)); //a little longer to grab the balls
        //ramp goes up once it's out of the flower, still before the turn
        run(drive.actionBuilder(new Pose2d(pickupSpot, farFlower.heading))
                .afterDisp(rampRetractDistance, () -> intakeBoard.rampUp())
                .strafeTo(farFlower.position) //backs out
                .strafeToLinearHeading(afterFlowerShot.position, afterFlowerShot.heading) //turns around on the way to the shot spot
                .build());
        turret.addAimOffset(flowerAimOffset); //after the second flower
        run(new SleepAction(farFlowerShotExtraWait));
        shoot();
    }

    //waits preShootWait, then runs the transfer for shootTime
    //after the cross the turret locks onto the april tags during the wait and the shot, odometry aim otherwise
    private void shoot() {
        tagAim = crossed;
        run(new SleepAction(preShootWait));
        transfer = true;
        run(new SleepAction(shootTime));
        transfer = false;
        tagAim = false;
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
            double target = Flywheel.shotVelocity;
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

    //runs the action and looks for the hive tag during the first scanTime
    //sees it: stops right there and returns true, doesn't see it: intake turns off and the action finishes, returns false
    private boolean runAndScan(Action action) {
        ElapsedTime scanTimer = new ElapsedTime();
        ElapsedTime seenTimer = new ElapsedTime();
        boolean seeing = false;
        boolean scanning = true;
        boolean running = true;
        while (opModeIsActive() && running) {
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);

            flywheel.update(Flywheel.shotVelocity, battery.getVoltage());
            Pose2d pose = drive.localizer.getPose();
            turret.aimAtGoal(pose, new PoseVelocity2d(new Vector2d(0, 0), 0));
            intakeBoard.runIntake(intake ? 1 : 0);
            transferBoard.run(transfer ? 1 : 0);
            dashboard.sendTelemetryPacket(packet);

            if (scanning) {
                if (scanTimer.seconds() >= scanTime) {
                    scanning = false;
                    intake = false; //no tag, it's crossing now
                } else if (turret.seesGoalTag(pose)) {
                    //the tag has to stay in view the whole tagConfirmTime, one flicker doesn't count
                    if (!seeing) {
                        seeing = true;
                        seenTimer.reset();
                    }
                    if (seenTimer.seconds() >= tagConfirmTime) {
                        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); //stops to shoot
                        return true;
                    }
                } else {
                    seeing = false;
                }
            }
        }
        return false;
    }

    //how long a built path takes, road runner wraps paths in a sequential action so this looks inside
    private static double moveTime(Action action) {
        if (action instanceof MecanumDrive.FollowTrajectoryAction) {
            return ((MecanumDrive.FollowTrajectoryAction) action).timeTrajectory.duration;
        }
        if (action instanceof SequentialAction) {
            double total = 0;
            for (Action a : ((SequentialAction) action).getInitialActions()) {
                total += moveTime(a);
            }
            return total;
        }
        return 0;
    }

    private void runRampEarly(double early, Action action) {
        runRampEarly(action, early);
    }

    //same as run, but the ramp starts dropping early seconds before the move ends
    private void runRampEarly(Action action, double early) {
        double dropAt = Math.max(0, moveTime(action) - early);
        ElapsedTime moveTimer = new ElapsedTime();
        boolean dropped = false;
        boolean running = true;
        while (opModeIsActive() && running) {
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);
            if (!dropped && moveTimer.seconds() >= dropAt) {
                intakeBoard.rampDown();
                dropped = true;
            }

            flywheel.update(Flywheel.shotVelocity, battery.getVoltage());
            Pose2d pose = drive.localizer.getPose();
            turret.aimAtGoal(pose, new PoseVelocity2d(new Vector2d(0, 0), 0));
            intakeBoard.runIntake(intake ? 1 : 0);
            transferBoard.run(transfer ? 1 : 0);
            dashboard.sendTelemetryPacket(packet);
        }
        intakeBoard.rampDown(); //makes sure it's down even if the move was cut short
    }

    //runs one action until it's done, keeps the flywheel updated like version 1
    private void run(Action action) {
        boolean running = true;
        while (opModeIsActive() && running) {
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);

            double flywheelTarget = Flywheel.shotVelocity;
            flywheel.update(flywheelTarget, battery.getVoltage());
            Pose2d pose = drive.localizer.getPose();
            if (!(tagAim && turret.aimAtTag(pose))) {
                turret.aimAtGoal(pose, new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
            intakeBoard.runIntake(intake ? 1 : 0);
            transferBoard.run(transfer ? 1 : 0);

            packet.put("Velocity", flywheel.returnVel());
            packet.put("targetVelocity", flywheelTarget);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
