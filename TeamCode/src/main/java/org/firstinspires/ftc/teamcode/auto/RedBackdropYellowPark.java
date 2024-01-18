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
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmDownAuto;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmUpAuto;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.intake.RetractPurpleCommand;
import org.firstinspires.ftc.teamcode.commands.vision.StopStreamingCommand;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.CenterStageVisionProcessor;

@Autonomous(group = "drive")
public class RedBackdropYellowPark extends AutoOpBase {

    private BotBuildersMecanumDrive robot;
    private DriveSubsystem drive;

    private VisionSubsystem visionSubsystem;

    private RobotStateSubsystem state;

    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;

    private VerticalSlideSubsystem verticalSubsystem;


    private HorizontalSlideSubsystem horizontalSlideSubsystem;



    private TrajectorySequenceFollowerCommand forwardFollower;
    private TrajectorySequenceFollowerCommand leftFollower;
    private TrajectorySequenceFollowerCommand rightFollower;



    @Override
    public void initialize() {
        robot = new BotBuildersMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(
                robot, null, telemetry);

        state = new RobotStateSubsystem();

        intakeSubsystem = new IntakeSubsystem(hardwareMap, state);
        visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
        armSubsystem = new ArmSubsystem(hardwareMap, state);
        verticalSubsystem = new VerticalSlideSubsystem(hardwareMap, state);
        horizontalSlideSubsystem = new HorizontalSlideSubsystem(hardwareMap, state, intakeSubsystem);

        //Set the starting position of the robot
        Pose2d startingPosition = new Pose2d(15, -62, Math.toRadians(90));

        drive.setPoseEstimate(startingPosition);

        //standard random forward movement.
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(61, -23.5, Math.toRadians(180)))
                .build();

        //simple movement to move close to the backdrop
        TrajectorySequence movePixelToCenter = drive.trajectorySequenceBuilder(moveForward.end())
                .lineToSplineHeading(new Pose2d(30,-16, Math.toRadians(180)))
                .build();


        //this is our standard left hand random move
        TrajectorySequence moveToLeft = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(61, -15.5, Math.toRadians(180)))
                .build();


        //this is our starting right hand random move
        TrajectorySequence moveToRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(61,-27.5, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToPixelLeft = drive.trajectorySequenceBuilder(moveToLeft.end())
                .lineToSplineHeading(new Pose2d(20,-31, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(17.5,-22, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        TrajectorySequence moveToPixelRight = drive.trajectorySequenceBuilder(moveToRight.end())
                .lineToSplineHeading(new Pose2d(40,-20, Math.toRadians(180)))
                .build();

        TrajectorySequence moveIntoCenterPosition = drive.trajectorySequenceBuilder(movePixelToCenter.end())
                .lineToSplineHeading(new Pose2d(55, -60, Math.toRadians(95))) //PARK
                .build();

        TrajectorySequence moveLeftIntoMovePos = drive.trajectorySequenceBuilder(moveToPixelLeft.end())
                .lineToSplineHeading(new Pose2d(55, -60, Math.toRadians(95))) //PARK
                .build();

        TrajectorySequence moveRightIntoMovePos = drive.trajectorySequenceBuilder(moveToPixelRight.end())
                .lineToSplineHeading(new Pose2d(50,-16, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(50, -60, Math.toRadians(95)))
                .lineToSplineHeading(new Pose2d(55, -60, Math.toRadians(95))) //PARK
                .build();



        forwardFollower = new TrajectorySequenceFollowerCommand(drive, moveForward);
        leftFollower = new TrajectorySequenceFollowerCommand(drive, moveToLeft);
        rightFollower = new TrajectorySequenceFollowerCommand(drive, moveToRight);

        TrajectorySequenceFollowerCommand movePixelToCenterFollower = new TrajectorySequenceFollowerCommand(drive, movePixelToCenter);
        TrajectorySequenceFollowerCommand movePixelToLeftFollower = new TrajectorySequenceFollowerCommand(drive, moveToPixelLeft);
        TrajectorySequenceFollowerCommand movePixelToRightFollower = new TrajectorySequenceFollowerCommand(drive, moveToPixelRight);


        TrajectorySequenceFollowerCommand moveToCenterFollower = new TrajectorySequenceFollowerCommand(drive, moveIntoCenterPosition);
        TrajectorySequenceFollowerCommand moveToLeftFollower = new TrajectorySequenceFollowerCommand(drive, moveLeftIntoMovePos);
        TrajectorySequenceFollowerCommand moveToRightFollower = new TrajectorySequenceFollowerCommand(drive, moveRightIntoMovePos);

        intakeSubsystem.ExtendPurple();

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new WaitCommand(15000),
                        new StopStreamingCommand(visionSubsystem),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                leftFollower,
                                                new SequentialCommandGroup(
                                                        new WaitCommand(500),
                                                        new ArmUpAuto(armSubsystem, verticalSubsystem, state)
                                                )
                                        ),
                                        new WaitCommand(500),
                                        new DropPixelCommand(armSubsystem),
                                        new WaitCommand(250),
                                        new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                        new WaitCommand(500),
                                        movePixelToLeftFollower,
                                        new RetractPurpleCommand(intakeSubsystem),
                                        moveToLeftFollower

                                ),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                            new ParallelCommandGroup(
                                                    rightFollower,
                                                    new SequentialCommandGroup(
                                                            new WaitCommand(500),
                                                            new ArmUpAuto(armSubsystem, verticalSubsystem, state)
                                                    )
                                            ),
                                            new WaitCommand(500),
                                            new DropPixelCommand(armSubsystem),
                                            new WaitCommand(250),
                                            new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                            new WaitCommand(500),
                                            movePixelToRightFollower,
                                            new RetractPurpleCommand(intakeSubsystem),
                                            moveToRightFollower

                                        ),
                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        forwardFollower,
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(500),
                                                                new ArmUpAuto(armSubsystem, verticalSubsystem, state)
                                                        )
                                                ),
                                                new WaitCommand(500),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(250),
                                                new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(500),
                                                movePixelToCenterFollower,
                                                new RetractPurpleCommand(intakeSubsystem),
                                                moveToCenterFollower
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
