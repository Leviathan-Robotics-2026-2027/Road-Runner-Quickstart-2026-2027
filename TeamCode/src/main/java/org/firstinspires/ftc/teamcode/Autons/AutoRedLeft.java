package org.firstinspires.ftc.teamcode.Autons;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ProgrammingBoards.Flywheel;

@Config
@Autonomous
public class AutoRedLeft extends LinearOpMode {
    //flywheel speed from 0 to 1, 1 is fullSpeedVelocity
    public static double flywheelSpeed = 0; //0 keeps it off for now
    public static double fullSpeedVelocity = 1000; //ticks per second at speed 1

    private Flywheel flywheel;
    private VoltageSensor battery;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        flywheel = new Flywheel(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();
        dashboard = FtcDashboard.getInstance();

        Pose2d startPose = new Pose2d(58, -12, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        //paths from meep meep, each one starts where the last one ends
        Pose2d pose1 = new Pose2d(24, -58, Math.toRadians(-90));
        Pose2d pose2 = new Pose2d(-56, -24, Math.toRadians(-180));
        Pose2d pose3 = new Pose2d(-35, -60, Math.toRadians(-90));
        Pose2d pose4 = new Pose2d(58, -12, Math.toRadians(-180));
        Pose2d pose5 = new Pose2d(60, -63, Math.toRadians(0));

        Action path1 = drive.actionBuilder(startPose)
                .setTangent(Math.toRadians(190))
                .splineToLinearHeading(pose1, Math.toRadians(-110))
                .build();
        Action path2 = drive.actionBuilder(pose1)
                .setTangent(Math.toRadians(145))
                .splineToLinearHeading(pose2, Math.toRadians(-205))
                .build();
        Action path3 = drive.actionBuilder(pose2)
                .setTangent(Math.toRadians(-85))
                .splineToSplineHeading(pose3, Math.toRadians(-90))
                .build();
        Action path4 = drive.actionBuilder(pose3)
                .setTangent(Math.toRadians(10))
                .splineToLinearHeading(pose4, Math.toRadians(40))
                .build();
        Action path5 = drive.actionBuilder(pose4)
                .setTangent(Math.toRadians(-150))
                .splineToSplineHeading(pose5, Math.toRadians(0))
                .build();
        Action path6 = drive.actionBuilder(pose5)
                .strafeToLinearHeading(new Vector2d(58, -12), Math.toRadians(0))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        run(path1);
        run(new SleepAction(1));
        run(path2);
        run(new SleepAction(2));
        run(path3);
        run(new SleepAction(0.5));
        run(path4);
        run(new SleepAction(0.5));
        run(path5);
        run(new SleepAction(0.5));
        run(path6);

        flywheel.update(0, battery.getVoltage());
    }

    //runs one action until it's done, keeps the flywheel updated like version 1
    private void run(Action action) {
        boolean running = true;
        while (opModeIsActive() && running) {
            TelemetryPacket packet = new TelemetryPacket();
            running = action.run(packet);

            double flywheelTarget = flywheelSpeed * fullSpeedVelocity;
            flywheel.update(flywheelTarget, battery.getVoltage());

            packet.put("Velocity", flywheel.returnVel());
            packet.put("targetVelocity", flywheelTarget);
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
