package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import javax.imageio.ImageIO;
import java.io.File;
import java.io.IOException;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(75, 75, Math.toRadians(270), Math.toRadians(270), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(58, -12, Math.toRadians(180)))
                //.strafeToLinearHeading(new Vector2d(24, -58), Math.toRadians(-90))
                .strafeTo(new Vector2d(52, -12))
                .strafeTo(new Vector2d(52, -12)) //6 inches forward
                .waitSeconds(2.7) //shooting
                .splineToLinearHeading(new Pose2d(24, -45, Math.toRadians(-90)), Math.toRadians(-110)) //stops facing the flower
                .waitSeconds(0.25) //ramp drops
                .strafeTo(new Vector2d(24, -58)) //drives forward into the flower
                .waitSeconds(0.65) //grabs the balls
                .strafeTo(new Vector2d(24, -45)) //backs up, ramp goes up
                .splineToLinearHeading(new Pose2d(52, -12, Math.toRadians(180)), Math.atan2(-12 - -45, 52 - 24)) //curves back to where it shot first, no stop
                .waitSeconds(0.75) //scans for a tag
                .waitSeconds(1.75) //shoots if it saw one
                .waitSeconds(0.75)
                .strafeToLinearHeading(new Vector2d(-20, -10), Math.toRadians(180), new TranslationalVelConstraint(75), new ProfileAccelConstraint(-45, 75))
                //second strafe as a spline so it rounds the corner without stopping
                .splineToLinearHeading(new Pose2d(-48, -30, Math.toRadians(180)), Math.atan2(-30 - -10, -48 - -20), new TranslationalVelConstraint(75), new ProfileAccelConstraint(-45, 75))
                //.splineToLinearHeading(new Pose2d(24, -36, Math.toRadians(-180)), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(24, -44), Math.toRadians(-180), new TranslationalVelConstraint(75), new ProfileAccelConstraint(-45, 75))
//                .splineToLinearHeading(new Pose2d(-48, -24, Math.toRadians(-180)), Math.toRadians(180), new TranslationalVelConstraint(75), new ProfileAccelConstraint(-45, 75)) //gentle curve
                .waitSeconds(0.25) //ramp drops
                .strafeTo(new Vector2d(-61, -30)) //moves 13 into the flower
                .waitSeconds(0.75) //grabs the balls
                .strafeTo(new Vector2d(-48, -30)) //backs out, ramp goes up
                .strafeToLinearHeading(new Vector2d(-58, -18), Math.toRadians(0)) //turns around on the way to the shot spot
                .waitSeconds(1.75)
                .strafeToLinearHeading(new Vector2d(-41, -60), Math.toRadians(270))
                .strafeTo(new Vector2d(-41, -72)) //pushes into the wall
                .waitSeconds(0.5) //grabs
                .strafeTo(new Vector2d(-41, -60)) //back the way it came
                .strafeToLinearHeading(new Vector2d(-58, -18), Math.toRadians(0))
                .waitSeconds(1.75) //shoots
                //if the hive tipped, the spline above goes to (-58, -18) facing 0 instead and it shoots there
                //.splineToLinearHeading(new Pose2d(-58, -18, Math.toRadians(0)), Math.toRadians(180))
                //.waitSeconds(2)
//                .splineToSplineHeading(new Pose2d(-35, -60, Math.toRadians(-90)), Math.toRadians(-90))
//                .waitSeconds(0.5)
//                .splineToLinearHeading(new Pose2d(58, -12, Math.toRadians(-180)), Math.toRadians(40))
//                .waitSeconds(0.5)
//                .splineToSplineHeading(new Pose2d(60, -63, Math.toRadians(0)), Math.toRadians(0))
//                .waitSeconds(0.5)
//                .strafeToLinearHeading(new Vector2d(58, -12), Math.toRadians(0))
                .build());

        try {
            // Corrected: Uses the full path AND adds the .png extension
            String path = System.getProperty("user.home") + "/Downloads/biobuzz-field-meepmeep.png";

            meepMeep.setBackground(ImageIO.read(new File(path)))
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        } catch (IOException e) {
            System.out.println("Could not find the image file in your Downloads folder!");
            e.printStackTrace();
        }
    }
}