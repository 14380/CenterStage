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
import org.firstinspires.ftc.teamcode.commands.arm.RotateTransferRightWhiteCommand;
import org.firstinspires.ftc.teamcode.commands.arm.SinglePixelDropAutoCommand;
import org.firstinspires.ftc.teamcode.commands.arm.SinglePixelDropCommand;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmDownAuto;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmUpLeftAuto;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmUpLeftAutoPos0;
import org.firstinspires.ftc.teamcode.commands.autogroup.AutoIntake;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeAdvanceCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeReverseCommand;
import org.firstinspires.ftc.teamcode.commands.intake.RetractPurpleCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.Pos0ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.Pos1ExtendCommand;
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
public class RedAudienceSinglePickup extends AutoOpBase {

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
        armSubsystem = new ArmSubsystem(hardwareMap, state);
        verticalSlideSubsystem = new VerticalSlideSubsystem(hardwareMap, state);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(-36, -62, Math.toRadians(90));

        drive.setPoseEstimate(startingPosition);

        //standard random forward movement.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)

                //move back ready to make first move
                .lineToSplineHeading(new Pose2d(-30, -33, Math.toRadians(90)))
                .build();

        TrajectorySequence moveForward2 = drive.trajectorySequenceBuilder(moveForward.end())

                //move back ready to make first move
                .lineToSplineHeading(new Pose2d(-42, -55, Math.toRadians(90)))
                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(-42,-1, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-48, -1, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-41, -1, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();


        //this is our standard left hand random move
        TrajectorySequence moveToLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(-38, -38, Math.toRadians(110)))

                .build();

        TrajectorySequence moveToLeft2 = drive.trajectorySequenceBuilder(moveToLeft.end())

                .lineToSplineHeading(new Pose2d(-30, -42, Math.toRadians(90)))
                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(-42,-1, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-48, -1, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-41, -1, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();


        //this is our starting right hand random move
        TrajectorySequence moveToRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(-28, -35, Math.toRadians(65)))

                .build();

        //this is our starting right hand random move
        TrajectorySequence moveToRight2 = drive.trajectorySequenceBuilder(moveToRight.end())

                .lineToSplineHeading(new Pose2d(-42, -42, Math.toRadians(90)))

                //move to in front of the stack
                .lineToSplineHeading(new Pose2d(-42,-1, Math.toRadians(180)))

                .lineToSplineHeading(new Pose2d(-48, -1, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-41, -1, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        //these are the three parking positions at the rear of the field
        //duplicated for each location, the starting paths are very similar.
        TrajectorySequence moveToBackDropParkRight = drive.trajectorySequenceBuilder(moveToRight2.end())
                .lineToSplineHeading(new Pose2d(58, -5, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropParkLeft = drive.trajectorySequenceBuilder(moveToLeft2.end())
                .lineToSplineHeading(new Pose2d(58, -5, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropParkCenter = drive.trajectorySequenceBuilder(moveForward2.end())
                .lineToSplineHeading(new Pose2d(58, -5, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropSideGameLeft = drive.trajectorySequenceBuilder(moveToBackDropParkCenter.end())
                .lineToSplineHeading(new Pose2d(66, -7, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropSideGameRight = drive.trajectorySequenceBuilder(moveToBackDropParkCenter.end())
                .lineToSplineHeading(new Pose2d(66, -22, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToBackDropSideGameCenter = drive.trajectorySequenceBuilder(moveToBackDropParkCenter.end())
                .lineToSplineHeading(new Pose2d(66, -12, Math.toRadians(180)))
                .build();

        //Orange sidegame - now do the white sidegame.
        TrajectorySequence moveToBackDropSideGameCenterOrange = drive.trajectorySequenceBuilder(moveToBackDropSideGameCenter.end())
                .lineToSplineHeading(new Pose2d(66, -9, Math.toRadians(180)))
                .build();

        TrajectorySequence moveOffBackDropCenter = drive.trajectorySequenceBuilder(moveToBackDropSideGameCenterOrange.end())
                .lineToSplineHeading(new Pose2d(58, -5, Math.toRadians(180)))
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

        TrajectorySequenceFollowerCommand moveOffBackdropCenter = new TrajectorySequenceFollowerCommand(drive, moveOffBackDropCenter);
        TrajectorySequenceFollowerCommand moveOffBackdropCenterRight = new TrajectorySequenceFollowerCommand(drive, moveOffBackDropCenter);
        TrajectorySequenceFollowerCommand moveOffBackdropLeft = new TrajectorySequenceFollowerCommand(drive, moveOffBackDropCenter);

        TrajectorySequenceFollowerCommand moveToBackDropSideGameCenterFollower = new TrajectorySequenceFollowerCommand(drive,moveToBackDropSideGameCenter );
        TrajectorySequenceFollowerCommand moveToBackDropSideGameRightFollower = new TrajectorySequenceFollowerCommand(drive, moveToBackDropSideGameRight);
        TrajectorySequenceFollowerCommand moveToBackDropSideGameLeftFollower = new TrajectorySequenceFollowerCommand(drive, moveToBackDropSideGameLeft);

        TrajectorySequenceFollowerCommand moveToSideForExtraWhite = new TrajectorySequenceFollowerCommand(drive, moveToBackDropSideGameCenterOrange);
        //wait for the op mode to start, then execute our paths.

        intake.ExtendPurple();

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new StopStreamingCommand(visionSubsystem),
                        //new WaitCommand(5000),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        leftFollower,
                                        new RetractPurpleCommand(intake),
                                        new WaitCommand(500),
                                        left2Follower,
                                        new AutoIntake(intake, armSubsystem),
                                        backFollowerLeft,
                                        new ParallelCommandGroup(
                                                new SequentialCommandGroup(
                                                        new MiddleArmUpCommand(armSubsystem),
                                                        new Pos1ExtendCommand(verticalSlideSubsystem)
                                                )
                                        ),
                                        new ArmUpLeftAutoPos0(armSubsystem, verticalSlideSubsystem, state),
                                        new WaitCommand(500),
                                        moveToBackDropSideGameLeftFollower,
                                        new DropPixelCommand(armSubsystem),
                                        new PosAutoExExtendCommand(verticalSlideSubsystem),
                                        //new RotateTransferRightWhiteCommand(armSubsystem),
                                        new WaitCommand(500),
                                        new DropPixelCommand(armSubsystem),
                                        new WaitCommand(500),
                                        moveOffBackdropLeft,
                                        new ArmDownAuto(armSubsystem, verticalSlideSubsystem, state)

                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                            rightFollower,
                                                new RetractPurpleCommand(intake),
                                                new WaitCommand(500),
                                                right2Follower,
                                                new AutoIntake(intake, armSubsystem),
                                                backFollowerRight,
                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new MiddleArmUpCommand(armSubsystem),
                                                                new Pos1ExtendCommand(verticalSlideSubsystem)
                                                        )
                                                ),
                                                new ArmUpLeftAutoPos0(armSubsystem, verticalSlideSubsystem, state),
                                                new WaitCommand(500),
                                                moveToBackDropSideGameRightFollower,
                                                new PosAutoExExtendCommand(verticalSlideSubsystem),
                                               // new RotateTransferRightWhiteCommand(armSubsystem),
                                                new WaitCommand(500),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(500),
                                                moveOffBackdropCenterRight,
                                                new ArmDownAuto(armSubsystem, verticalSlideSubsystem, state)

                                        ),
                                        new SequentialCommandGroup(
                                            forwardFollower,
                                                new RetractPurpleCommand(intake),
                                                new WaitCommand(500),
                                                forward2Follower,
                                                new AutoIntake(intake, armSubsystem),
                                                backFollowerCenter,
                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new MiddleArmUpCommand(armSubsystem),
                                                                new Pos0ExtendCommand(verticalSlideSubsystem)
                                                        )
                                                ),
                                                new ArmUpLeftAutoPos0(armSubsystem, verticalSlideSubsystem, state),
                                                new WaitCommand(500),
                                                moveToBackDropSideGameCenterFollower,
                                                moveToSideForExtraWhite,
                                                new PosAutoExExtendCommand(verticalSlideSubsystem),
                                               // new RotateTransferRightWhiteCommand(armSubsystem),
                                                new WaitCommand(500),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(500),
                                                moveOffBackdropCenter,
                                                new ArmDownAuto(armSubsystem, verticalSlideSubsystem, state)

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
