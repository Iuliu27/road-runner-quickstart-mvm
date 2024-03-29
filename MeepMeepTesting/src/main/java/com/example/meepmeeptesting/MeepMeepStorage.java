package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepStorage {
    MeepMeep meepMeep = new MeepMeep(600);

    Pose2d stackPose = new Pose2d(-57, -36, Math.toRadians(180)),
            middleBackboardPose = new Pose2d(49, -36, Math.toRadians(0)),
            rightBackboardPose = new Pose2d(49, -42, Math.toRadians(0)),
            leftBackboardPose = new Pose2d(49, -30, Math.toRadians(0));

    RoadRunnerBotEntity AutoStangaRedBottom = new DefaultBotBuilder(meepMeep) //todo: done
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                            driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58, Math.toRadians(90)))

                                    //RIGHT
                                    .splineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(60)), Math.toRadians(60))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-57, -36), Math.toRadians(180))

                                    //MIDDLE
//                            .splineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)), Math.toRadians(80))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-57, -36), Math.toRadians(180))

                                    //LEFT
//                            .splineToLinearHeading(new Pose2d(-43, -33, Math.toRadians(110)), Math.toRadians(120))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-57, -36), Math.toRadians(180))


                                    .waitSeconds(0) //wait half  a sec!! (maybe?)
                                    //stackToMiddle
//                            .setReversed(false)
//                            .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))
                                    //stackToLeft
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))


                                    //middleToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(180))
                                    //stackToRight
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                                    //rightToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(180))
                                    //stackToRight
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                                    //parkBottom
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(49, -60, Math.toRadians(0)), Math.toRadians(-90))
                                    //.splineToLinearHeading(new Pose2d(57.5, -60, Math.toRadians(0)), Math.toRadians(0))
                                    .build()
            );
    RoadRunnerBotEntity AutoStangaRedUpper = new DefaultBotBuilder(meepMeep) //todo: done
            // Set bot constraints: maxVel 45, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(100, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                            driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58, Math.toRadians(90)))

                                    //RIGHT
                                    .splineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(60)), Math.toRadians(60))
                                    .setReversed(true)
                                    .splineTo(new Vector2d(-40, -36), Math.toRadians(180))
                                    .splineToConstantHeading(new Vector2d(-57, -25), Math.toRadians(180))

                                    //MIDDLE
//                            .splineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)), Math.toRadians(80))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-40, -38), Math.toRadians(180))
//                            .splineToConstantHeading(new Vector2d(-57, -25), Math.toRadians(180))

                                    //LEFT
//                            .splineToLinearHeading(new Pose2d(-43, -33, Math.toRadians(110)), Math.toRadians(120))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-52, -40), Math.toRadians(180))
//                            .splineToConstantHeading(new Vector2d(-57, -25), Math.toRadians(90))


                                    //stackToMiddleFirst
//                            .setReversed(false)
//                            .setTangent(Math.toRadians(70))
//                            .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))
                                    //stackToRightFirst
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(70))
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                                    //stackToLeftFirst
//                            .setReversed(false)
//                            .setTangent(Math.toRadians(70))
//                            .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))


                                    //middleToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    //stackToLeft
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))
                                    //leftToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    //stackToLeft
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))
                                    //parkUpper
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(49, -19, Math.toRadians(0)), Math.toRadians(90))
                                    //.splineToLinearHeading(new Pose2d(59, -10, Math.toRadians(0)), Math.toRadians(0))
                                    .build()
            );


    RoadRunnerBotEntity AutoDreaptaRedBottom = new DefaultBotBuilder(meepMeep) //todo: done
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                            driveShim.trajectorySequenceBuilder(new Pose2d(-34.5 + 2 * 24, -58, Math.toRadians(90)))

                                    //RIGHT
                                    .splineToLinearHeading(new Pose2d(18, -33, Math.toRadians(60)), Math.toRadians(60))
                                    .setReversed(true)
                                    .splineToLinearHeading(rightBackboardPose, Math.toRadians(0))

                                    //MIDDLE
//                            .splineToLinearHeading(new Pose2d(-32.5 + 2*24, -33, Math.toRadians(80)), Math.toRadians(80))
//                            .setReversed(true)
//                            .splineToLinearHeading(middleBackboardPose, Math.toRadians(0))


                                    //LEFT
//                                    .splineTo(new Vector2d(13.5, -53), Math.toRadians(90))
//                                    .splineTo(new Vector2d(4.5, -33), Math.toRadians(120))
//                                    .setReversed(true)
//                                    .splineToLinearHeading(leftBackboardPose, Math.toRadians(0))


                                    //backboardToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(110))


                                    //stackToRight
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(-70))
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(rightBackboardPose, Math.toRadians(0))
                                    //rightToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(110))
                                    //stackToRight
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(-70))
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(rightBackboardPose, Math.toRadians(0))
                                    //rightToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(110))
                                    //stackToRight
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(-70))
                                    .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(rightBackboardPose, Math.toRadians(0))
                                    //parkBottom
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(new Pose2d(49, -55, Math.toRadians(0)), Math.toRadians(-90))
                                    .build()
            );
    RoadRunnerBotEntity AutoDreaptaRedUpper = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                            driveShim.trajectorySequenceBuilder(new Pose2d(-34.5 + 2 * 24, -58, Math.toRadians(90)))

                                    //RIGHT
//                            .splineToLinearHeading(new Pose2d(-30 + 2*24, -33, Math.toRadians(60)), Math.toRadians(60))
//                            .setReversed(true)
//                            .splineToLinearHeading(leftBackboardPose, Math.toRadians(0))

                                    //MIDDLE
//                            .splineToLinearHeading(new Pose2d(-32.5 + 2*24, -33, Math.toRadians(80)), Math.toRadians(80))
//                            .setReversed(true)
//                            .splineToLinearHeading(middleBackboardPose, Math.toRadians(0))


                                    //LEFT
                                    .splineTo(new Vector2d(13.5, -53), Math.toRadians(90))
                                    .splineTo(new Vector2d(4.5, -33), Math.toRadians(120))
                                    .setReversed(true)
                                    .splineToLinearHeading(leftBackboardPose, Math.toRadians(0))

                                    //middleToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    //stackToLeft
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))
                                    //leftToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    //stackToLeft
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))
                                    //leftToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(0)), Math.toRadians(180))
                                    //stackToLeft
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))
                                    //parkUpper
                                    .setReversed(false)
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(new Pose2d(49, -19, Math.toRadians(0)), Math.toRadians(90))
                                    .build()
            );

    RoadRunnerBotEntity Auto2PixelStangaBlue = new DefaultBotBuilder(meepMeep) //todo: done
            // Set bot constraints: maxVel 45, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(100, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58, Math.toRadians(90)))
                            .build()
            );


}
