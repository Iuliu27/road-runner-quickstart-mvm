package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepStorage1 {
    MeepMeep meepMeep = new MeepMeep(700);
    RoadRunnerBotEntity AutoRedStanga3Up = new DefaultBotBuilder(meepMeep)   //Gata
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))

                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
                            .waitSeconds(0.2)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(90))
                            .waitSeconds(0.7) //ia un pixel
                            //1
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-38.39, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -41.31,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .setReversed(false)
                            .waitSeconds(0.7)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .setReversed(false)
                            .waitSeconds(0.7)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            //de final
                            .setReversed(true)
                            .setTangent(Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(48,-15,Math.toRadians(0)),Math.toRadians(90))
                            .build()
            );

    RoadRunnerBotEntity AutoRedStanga3Down = new DefaultBotBuilder(meepMeep) //Gata
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5  , -58 , Math.toRadians(90)))

                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
                            .waitSeconds(0.2)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(180))
                            .waitSeconds(0.7) //ia un pixel
                            //1
                            .setTangent(Math.toRadians(90))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(180))
                            .waitSeconds(0.7)
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(180))
                            .waitSeconds(0.7)
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)

                            //de final
                            .setReversed(true)
                            .setTangent(Math.toRadians(-90))
                            .splineToSplineHeading(new Pose2d(48,-56,Math.toRadians(0)),Math.toRadians(270))
                            .build()
            );

    RoadRunnerBotEntity AutoRedStanga1Up=new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))

                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-39,-36,Math.toRadians(120)),Math.toRadians(90))
                            .waitSeconds(0.2)
                            //.setReversed(true)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-37,-45,Math.toRadians(90)),Math.toRadians(90))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-58,-36,Math.toRadians(0)),Math.toRadians(90))
                            .waitSeconds(0.7) //ia un pixel
                            //1
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-49, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .setReversed(false)
                            .waitSeconds(0.7)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .setReversed(false)
                            .waitSeconds(0.7)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            //de final
                            .setReversed(true)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(48,-15,Math.toRadians(0)),Math.toRadians(90))
                            .build()
            );

    RoadRunnerBotEntity AutoRedStanga1Down=new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-39,-36,Math.toRadians(120)),Math.toRadians(90))
                            .waitSeconds(0.2)
                            //.setReversed(true)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-37,-45,Math.toRadians(90)),Math.toRadians(90))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-58,-36,Math.toRadians(0)),Math.toRadians(90))
                            .waitSeconds(0.7) //ia un pixel
                            //1
                            .setTangent(Math.toRadians(90))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-58,-36,Math.toRadians(0)),Math.toRadians(180))
                            .waitSeconds(0.7)
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-58,-36,Math.toRadians(0)),Math.toRadians(180))
                            .waitSeconds(0.7)
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)

                            //de final
                            .setReversed(true)
                            .setTangent(Math.toRadians(-90))
                            .splineToSplineHeading(new Pose2d(48,-56,Math.toRadians(0)),Math.toRadians(270))
                            .build()
            );


    RoadRunnerBotEntity AutoRedDreapta1Up=new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(11, -66 , Math.toRadians(90)))

                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(5.62,-37,Math.toRadians(115)),Math.toRadians(90))
                            .setTangent(Math.toRadians(0))
                            .setReversed(true)
                            .splineToSplineHeading(new Pose2d(21, -43, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -30, Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .setReversed(false)
                            .waitSeconds(0.7)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .setReversed(false)
                            .waitSeconds(0.7)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            //de final
                            .setReversed(true)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(48,-15,Math.toRadians(0)),Math.toRadians(90))
                            .build()
            );

    RoadRunnerBotEntity AutoRedDreapta1Down=new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(15, -62 , Math.toRadians(90)))
                            .setReversed(false)
                            .splineToSplineHeading(new Pose2d(23.00, -36.00, Math.toRadians(80.00)),Math.toRadians(80))
                            .setTangent(Math.toRadians(0))
                            .setReversed(true)

                            .splineToLinearHeading(new Pose2d(48, -30, Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7) //pune pixelul
                            .splineToLinearHeading(new Pose2d(2,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-54,-59,Math.toRadians(0)),Math.toRadians(180))
                            .build()
            );
    RoadRunnerBotEntity AutoTesting=new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                            //.setReversed(true)
                            .setReversed(false)
                            //.setTangent(Math.toRadians(-90))
                            .splineToLinearHeading(new Pose2d(48,-56,Math.toRadians(0)),Math.toRadians(0))
                            .build()
            );
    RoadRunnerBotEntity lidi=new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(11.38, -59.19, Math.toRadians(90.00)))
                            /*    .splineTo(new Vector2d(1.63, -30.01), Math.toRadians(90.00))
                                 .setReversed(true)
                                 .splineTo(new Vector2d(54.36, -34.61), Math.toRadians(0.00))
                                 .setReversed(false)
                                 .splineTo(new Vector2d(30.58, -12.18), Math.toRadians(180.00))


                                 .splineTo(new Vector2d(4.96, -41.08), Math.toRadians(110.00))
                                 .setReversed(false)
                                 .splineTo(new Vector2d(54.36, -34.61), Math.toRadians(0.00))
                                 .setReversed(true)
                                 .splineTo(new Vector2d(30.58, -12), Math.toRadians(180.00))
                                 .splineTo(new Vector2d(-60,-12),Math.toRadians(180))*/
                            .splineToLinearHeading(new Pose2d(-30 + 2*24, -33, Math.toRadians(60)), Math.toRadians(60))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))


                            /*.splineToLinearHeading(new Pose2d(10.83, -42.47, Math.toRadians(141.34)), Math.toRadians(141.34))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(50.91, -34.61, Math.toRadians(180.00)), Math.toRadians(180.00))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(33.46, -12.56, Math.toRadians(177.07)), Math.toRadians(177.07))

                            .splineToLinearHeading(new Pose2d(-54.55, -12.37, Math.toRadians(183.18)), Math.toRadians(183.18))
                    */
                            .build()


            );
    RoadRunnerBotEntity IulioCameraAuto = new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(11.34, 61.92, Math.toRadians(270.00))).splineTo(new Vector2d(0.59, 33.19), Math.toRadians(249.52))
                            .splineTo(new Vector2d(0.59, 33.19), Math.toRadians(249.52))
                            .lineTo(new Vector2d(19.24, 54.86))
                            .splineTo(new Vector2d(52.34, 41.25), Math.toRadians(0.00))
                            .build()




            );
    RoadRunnerBotEntity IulioCameraAuto2 = new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-36.38, -60.74, Math.toRadians(85.91)))
                            ///dreapta jos
                            .splineTo(new Vector2d(-26.80, -31.51), Math.toRadians(79.99))
                            .splineTo(new Vector2d(-61.25, -35.54), Math.toRadians(180.00))
                            .splineTo(new Vector2d(-40.58, -58.05), Math.toRadians(180.00))
                            .splineTo(new Vector2d(35.20, -58.73), Math.toRadians(180.00))
                            .splineTo(new Vector2d(49.48, -31.00), Math.toRadians(180.00))

                            .build()
            );



    RoadRunnerBotEntity LinieMijloc=new DefaultBotBuilder(meepMeep)    //Linie mijloc
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                            .splineToLinearHeading(new Pose2d(-35,-33,Math.toRadians(90)),Math.toRadians(90))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-35,-42,Math.toRadians(90)),Math.toRadians(90))
                            .build()
            );
    RoadRunnerBotEntity LinieStanga=new DefaultBotBuilder(meepMeep)    //Linie stanga
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-39,-36,Math.toRadians(120)),Math.toRadians(90))
                            .waitSeconds(0.2)
                            //.setReversed(true)
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-37,-45,Math.toRadians(90)),Math.toRadians(90))
                            .setReversed(false)
                            .build()
            );
    RoadRunnerBotEntity LinieDreapta=new DefaultBotBuilder(meepMeep)    //Linie dreapta
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                            /*.splineToLinearHeading(new Pose2d(-27,-32,Math.toRadians(60)),Math.toRadians(60))
                            .setTangent(Math.toRadians(-270))
                            .splineToLinearHeading(new Pose2d(-34.5,-58,Math.toRadians(90)),Math.toRadians(-270))
                            .setTangent(Math.toRadians(180))*/
                            .splineToLinearHeading(new Pose2d(-80,-56),Math.toRadians(180))
                            .build()
            );
    RoadRunnerBotEntity test=new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                            .splineToLinearHeading(new Pose2d(-35,-29,Math.toRadians(90)),Math.toRadians(90))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-35,-42,Math.toRadians(90)),Math.toRadians(90))
                            .setReversed(true)
                            .build()
            );

}
