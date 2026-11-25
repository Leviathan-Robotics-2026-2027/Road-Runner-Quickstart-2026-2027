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

        //just the cross to the other side, starts at the scan spot
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(52, -12, Math.toRadians(180)))
                .strafeToLinearHeading(new Vector2d(-20, -10), Math.toRadians(180), new TranslationalVelConstraint(63.75), new ProfileAccelConstraint(-38.25, 63.75))
                //second strafe as a spline so it rounds the corner without stopping
                .splineToLinearHeading(new Pose2d(-48, -30, Math.toRadians(0)), 2 * Math.atan2(-30 - -10, -48 - -20) - Math.atan2(-10 - -12, -20 - 52), new TranslationalVelConstraint(63.75), new ProfileAccelConstraint(-38.25, 63.75))
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