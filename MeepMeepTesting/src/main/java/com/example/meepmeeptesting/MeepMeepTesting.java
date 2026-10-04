package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
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
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                .strafeToLinearHeading(new Vector2d(-10, 6), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(22, 25, Math.toRadians(90)), Math.toRadians(0))

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