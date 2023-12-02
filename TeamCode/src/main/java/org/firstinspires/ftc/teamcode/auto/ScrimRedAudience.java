package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.intake.RetractPurpleCommand;
import org.firstinspires.ftc.teamcode.commands.vision.StopStreamingCommand;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.CenterStageVisionProcessor;

@Autonomous(group = "drive")
public class ScrimRedAudience extends AutoOpBase {

    private BotBuildersMecanumDrive robot;
    private DriveSubsystem drive;

    private VisionSubsystem visionSubsystem;

    private RobotStateSubsystem state;

    private IntakeSubsystem intake;


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

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(36, -62, Math.toRadians(90));

        drive.setPoseEstimate(startingPosition);

        //standard random forward movement.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)

                //move back ready to make first move
                .lineToSplineHeading(new Pose2d(42, -33, Math.toRadians(90)))
                .build();

        TrajectorySequence moveForward2 = drive.trajectorySequenceBuilder(moveForward.end())

                //move back ready to make first move
                .lineToSplineHeading(new Pose2d(30, -55, Math.toRadians(90)))
                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(30,0, Math.toRadians(180)))
                .build();


        //this is our standard left hand random move
        TrajectorySequence moveToLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(34, -38, Math.toRadians(110)))

                .build();

        TrajectorySequence moveToLeft2 = drive.trajectorySequenceBuilder(moveToLeft.end())

                .lineToSplineHeading(new Pose2d(42, -42, Math.toRadians(90)))
                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(30,0, Math.toRadians(180)))
                .build();


        //this is our starting right hand random move
        TrajectorySequence moveToRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(44, -35, Math.toRadians(65)))

                .build();

        //this is our starting right hand random move
        TrajectorySequence moveToRight2 = drive.trajectorySequenceBuilder(moveToRight.end())

                .lineToSplineHeading(new Pose2d(30, -42, Math.toRadians(90)))

                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(30,0, Math.toRadians(180)))
                .build();

        //these are the three parking positions at the rear of the field
        //duplicated for each location, the starting paths are very similar.
        TrajectorySequence moveToBackDropParkRight = drive.trajectorySequenceBuilder(moveToRight2.end())
                .lineToSplineHeading(new Pose2d(130, -5, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropParkLeft = drive.trajectorySequenceBuilder(moveToLeft2.end())
                .lineToSplineHeading(new Pose2d(130, -5, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropParkCenter = drive.trajectorySequenceBuilder(moveForward2.end())
                .lineToSplineHeading(new Pose2d(130, -5, Math.toRadians(180)))
                .build();


        forwardFollower = new TrajectorySequenceFollowerCommand(drive, moveForward);
        leftFollower = new TrajectorySequenceFollowerCommand(drive, moveToLeft);
        rightFollower = new TrajectorySequenceFollowerCommand(drive, moveToRight);

        backFollowerLeft = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkLeft);
        backFollowerRight = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkRight);
        backFollowerCenter = new TrajectorySequenceFollowerCommand(drive, moveToBackDropParkCenter);

        TrajectorySequenceFollowerCommand forward2Follower = new TrajectorySequenceFollowerCommand(drive, moveForward2);
        TrajectorySequenceFollowerCommand left2Follower = new TrajectorySequenceFollowerCommand(drive, moveToLeft2);
        TrajectorySequenceFollowerCommand right2Follower = new TrajectorySequenceFollowerCommand(drive, moveToRight2);

        //wait for the op mode to start, then execute our paths.

        intake.ExtendPurple();

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new StopStreamingCommand(visionSubsystem),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        leftFollower,
                                        new RetractPurpleCommand(intake),
                                        new WaitCommand(500),
                                        left2Follower,
                                        backFollowerLeft
                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                            rightFollower,
                                                new RetractPurpleCommand(intake),
                                                new WaitCommand(500),
                                                right2Follower,
                                                backFollowerRight
                                        ),
                                        new SequentialCommandGroup(
                                            forwardFollower,
                                                new RetractPurpleCommand(intake),
                                                new WaitCommand(500),
                                                forward2Follower,
                                                backFollowerCenter
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
