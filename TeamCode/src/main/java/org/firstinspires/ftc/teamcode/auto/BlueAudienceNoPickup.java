package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.arm.DropPixelCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmDownAuto;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmUpRightAutoPos0;
import org.firstinspires.ftc.teamcode.commands.autogroup.AutoIntake;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.intake.RetractPurpleCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.Pos0ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.PosAutoExExtendCommand;
import org.firstinspires.ftc.teamcode.commands.vision.StopStreamingCommand;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.CenterStageVisionProcessor;

@Autonomous(group = "drive")
public class BlueAudienceNoPickup extends AutoOpBase {

    private BotBuildersMecanumDrive robot;
    private DriveSubsystem drive;

    private VisionSubsystem visionSubsystem;

    private RobotStateSubsystem state;

    private IntakeSubsystem intake;

    private ArmSubsystem armSubsystem;

    private VerticalSlideSubsystem verticalSlideSubsystem;


    private TrajectorySequenceFollowerCommand forwardFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand rightFollower;

    private TrajectorySequenceFollowerCommand backFollowerLeft;
    private TrajectorySequenceFollowerCommand backFollowerRight;
    private TrajectorySequenceFollowerCommand backFollowerCenter;

    @Override
    public void initialize() {
        robot = new BotBuildersMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(
                robot, null, telemetry);

        state = new RobotStateSubsystem();

        intake = new IntakeSubsystem(hardwareMap, state);
        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        verticalSlideSubsystem = new VerticalSlideSubsystem(hardwareMap, state);
        armSubsystem = new ArmSubsystem(hardwareMap, state);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(-36, 62, Math.toRadians(270));

        drive.setPoseEstimate(startingPosition);

        //standard random forward movement.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)
                //push forward, slightly to the right of center
                .lineToSplineHeading(new Pose2d(-28,33, Math.toRadians(270)))
                .build();

        TrajectorySequence moveForward2 = drive.trajectorySequenceBuilder(moveForward.end())

                //move back ready to make first move
                .lineToSplineHeading(new Pose2d(-40, 55, Math.toRadians(270)))
                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(-42,2, Math.toRadians(180)))


                .build();


        //this is our standard left hand random move
        TrajectorySequence moveToLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(-28, 38, Math.toRadians(300)))
                .build();

        //this is our standard left hand random move
        TrajectorySequence moveToLeft2 = drive.trajectorySequenceBuilder(moveToLeft.end())

                .lineToSplineHeading(new Pose2d(-40, 55, Math.toRadians(270)))
                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(-45,0, Math.toRadians(180)))

                .build();


        //this is our starting right hand random move
        TrajectorySequence moveToRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(-42, 38, Math.toRadians(270)))
                .build();

        TrajectorySequence moveToRight2 = drive.trajectorySequenceBuilder(moveToRight.end())
                .lineToSplineHeading(new Pose2d(-33, 55, Math.toRadians(270)))
                .lineToSplineHeading(new Pose2d(-31, 25, Math.toRadians(270)))
                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(-45,5, Math.toRadians(180)))

                .build();

        //these are the three transient positions at the rear of the field
        //duplicated for each location, the starting paths are very similar.
        TrajectorySequence moveToBackDropParkRight = drive.trajectorySequenceBuilder(moveToRight2.end())
                .lineToSplineHeading(new Pose2d(59, 4, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropParkLeft = drive.trajectorySequenceBuilder(moveToLeft2.end())
                .lineToSplineHeading(new Pose2d(59, 4, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropParkCenter = drive.trajectorySequenceBuilder(moveForward2.end())
                .lineToSplineHeading(new Pose2d(59, 4, Math.toRadians(180)))
                .build();

        //location on the backdrop for the center pixel to be dropped
        TrajectorySequence moveToBackDropCenter = drive.trajectorySequenceBuilder(moveToBackDropParkCenter.end())
                .lineToSplineHeading(new Pose2d(65, 9.5, Math.toRadians(180)))
                .build();

        TrajectorySequence moveOffBackDropCenter = drive.trajectorySequenceBuilder(moveToBackDropCenter.end())
                .lineToSplineHeading(new Pose2d(59, 4, Math.toRadians(180)))
                .build();

        //parking location for center
        TrajectorySequence moveToParkCenter = drive.trajectorySequenceBuilder(moveToBackDropCenter.end())
                .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(180)))
                .build();
        //parking with the turn
        TrajectorySequence parkTurnCenter = drive.trajectorySequenceBuilder(moveToParkCenter.end())
                .lineToSplineHeading(new Pose2d(55, 12, Math.toRadians(270)))
                .build();

        //parking location for left
        TrajectorySequence moveToLeftPark = drive.trajectorySequenceBuilder(moveToBackDropParkLeft.end())
                .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(180)))
                .build();


        //location on the backdrop for the left pixel to be dropped.
        TrajectorySequence moveToBackDropLeft = drive.trajectorySequenceBuilder(moveToBackDropParkLeft.end())
                .lineToSplineHeading(new Pose2d(64, 17, Math.toRadians(180)))
                .build();

        //location on the backdrop for the right pixel to be dropped.
        TrajectorySequence moveToBackDropRight = drive.trajectorySequenceBuilder(moveToBackDropParkRight.end())
                .lineToSplineHeading(new Pose2d(65, 7, Math.toRadians(180)))
                .build();



        TrajectorySequence moveOffBackDropLeft = drive.trajectorySequenceBuilder(moveToBackDropLeft.end())
                .lineToSplineHeading(new Pose2d(59, 4, Math.toRadians(180)))
                .build();

        TrajectorySequence moveOffBackDropRight = drive.trajectorySequenceBuilder((moveToBackDropRight.end()))
                .lineToSplineHeading(new Pose2d(59, 4, Math.toRadians(180)))
                .build();

        //parking location for the right hand side
        TrajectorySequence moveToRightPark = drive.trajectorySequenceBuilder(moveOffBackDropRight.end())
                .lineToSplineHeading(new Pose2d(60, 0, Math.toRadians(180)))
                .build();


        forwardFollower = new TrajectorySequenceFollowerCommand(drive, moveForward);
        leftFollower = new TrajectorySequenceFollowerCommand(drive, moveToLeft);
        rightFollower = new TrajectorySequenceFollowerCommand(drive, moveToRight);

        //these paths get us from the audience to the backdrop.
        backFollowerLeft = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkLeft);
        backFollowerRight = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkRight);
        backFollowerCenter = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkCenter);

        TrajectorySequenceFollowerCommand moveToBackDropSideGameCenterFollower = new TrajectorySequenceFollowerCommand(drive, moveToBackDropCenter);
        TrajectorySequenceFollowerCommand moveOffBackdropCenterFollower = new TrajectorySequenceFollowerCommand(drive, moveOffBackDropCenter);


        TrajectorySequenceFollowerCommand moveToBackDropSideGameLeftFollower = new TrajectorySequenceFollowerCommand(drive, moveToBackDropLeft);
        TrajectorySequenceFollowerCommand moveOffBackdropLeftFollower = new TrajectorySequenceFollowerCommand(drive, moveOffBackDropLeft);

        TrajectorySequenceFollowerCommand moveToBackDropSideGameRightFollower = new TrajectorySequenceFollowerCommand(drive, moveToBackDropRight);
        TrajectorySequenceFollowerCommand moveOffBackdropRightFollower = new TrajectorySequenceFollowerCommand(drive, moveOffBackDropRight);

        TrajectorySequenceFollowerCommand moveForward1 = new TrajectorySequenceFollowerCommand(drive, moveForward2);
        TrajectorySequenceFollowerCommand moveRight2 = new TrajectorySequenceFollowerCommand(drive, moveToRight2);
        TrajectorySequenceFollowerCommand moveLeft2 = new TrajectorySequenceFollowerCommand(drive, moveToLeft2);

        TrajectorySequenceFollowerCommand centerParkFollower = new TrajectorySequenceFollowerCommand(drive, moveToParkCenter);
        TrajectorySequenceFollowerCommand leftParkFollower = new TrajectorySequenceFollowerCommand(drive, moveToLeftPark);
        TrajectorySequenceFollowerCommand rightParkFollower = new TrajectorySequenceFollowerCommand(drive, moveToRightPark);

        TrajectorySequenceFollowerCommand centerParkTurnFollower = new TrajectorySequenceFollowerCommand(drive, parkTurnCenter);
        TrajectorySequenceFollowerCommand rightParkTurnFollower = new TrajectorySequenceFollowerCommand(drive, parkTurnCenter);
        TrajectorySequenceFollowerCommand leftParkTurnFollower = new TrajectorySequenceFollowerCommand(drive, parkTurnCenter);


        intake.ExtendPurple();

        CommandScheduler.getInstance().schedule(
               // new WaitCommand(8000),
                new WaitUntilCommand(this::isStarted).andThen(
                        new WaitCommand(8000),
                        new StopStreamingCommand(visionSubsystem),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        leftFollower,
                                        new RetractPurpleCommand(intake),
                                        new WaitCommand(500),
                                        moveLeft2,

                                        backFollowerLeft,
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                        new MiddleArmUpCommand(armSubsystem),
                                                        new Pos0ExtendCommand(verticalSlideSubsystem)
                                                )
                                        ),
                                        new ArmUpRightAutoPos0(armSubsystem, verticalSlideSubsystem, state),
                                        new WaitCommand(500),
                                        moveToBackDropSideGameLeftFollower,
                                        new WaitCommand(500),
                                        new DropPixelCommand(armSubsystem),
                                        new WaitCommand(500),
                                        new PosAutoExExtendCommand(verticalSlideSubsystem),
                                        new WaitCommand(500),
                                        moveOffBackdropLeftFollower,
                                        new ArmDownAuto(armSubsystem, verticalSlideSubsystem, state),
                                        leftParkFollower,
                                        leftParkTurnFollower
                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                            rightFollower,
                                                new RetractPurpleCommand(intake),
                                                new WaitCommand(500),
                                                moveRight2,
                                                backFollowerRight,
                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new MiddleArmUpCommand(armSubsystem),
                                                                new Pos0ExtendCommand(verticalSlideSubsystem)
                                                        )
                                                ),
                                                new ArmUpRightAutoPos0(armSubsystem, verticalSlideSubsystem, state),
                                                new WaitCommand(500),
                                                moveToBackDropSideGameRightFollower,
                                                new WaitCommand(500),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(500),
                                                new PosAutoExExtendCommand(verticalSlideSubsystem),
                                                new WaitCommand(500),
                                                moveOffBackdropRightFollower,
                                                new ArmDownAuto(armSubsystem, verticalSlideSubsystem, state),
                                                rightParkFollower,
                                                rightParkTurnFollower
                                        ),
                                        new SequentialCommandGroup(
                                            forwardFollower,
                                                new RetractPurpleCommand(intake),
                                                new WaitCommand(500),
                                                moveForward1,
                                                backFollowerCenter,
                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new MiddleArmUpCommand(armSubsystem),
                                                                new Pos0ExtendCommand(verticalSlideSubsystem)
                                                        )
                                                ),
                                                new ArmUpRightAutoPos0(armSubsystem, verticalSlideSubsystem, state),
                                                new WaitCommand(500),
                                                moveToBackDropSideGameCenterFollower,
                                                new WaitCommand(500),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(500),
                                                new PosAutoExExtendCommand(verticalSlideSubsystem),
                                                new WaitCommand(500),
                                                moveOffBackdropCenterFollower,
                                                new ArmDownAuto(armSubsystem, verticalSlideSubsystem, state),
                                                centerParkFollower,
                                                centerParkTurnFollower

                                        ),
                                        ()->{
                                            return visionSubsystem.getPosition() == CenterStageVisionProcessor.StartingPosition.RIGHT;
                                        }
                                ),
                                ()-> { return visionSubsystem.getPosition() == CenterStageVisionProcessor.StartingPosition.LEFT;
                                })


                ));

    }

    @Override
    public void run(){

        CommandScheduler.getInstance().run();

    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart(){

    }
}
