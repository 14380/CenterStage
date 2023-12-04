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
import org.firstinspires.ftc.teamcode.commands.arm.LockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.arm.UnlockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmDownAuto;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmUpAuto;
import org.firstinspires.ftc.teamcode.commands.autogroup.ArmUpLeftAuto;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
import org.firstinspires.ftc.teamcode.commands.intake.RetractPurpleCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.Pos1ExtendCommand;
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
public class RedBackdropMiddleDouble extends AutoOpBase {

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
        horizontalSlideSubsystem = new HorizontalSlideSubsystem(hardwareMap, state);

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
                .lineToSplineHeading(new Pose2d(61, -18, Math.toRadians(180)))
                .build();


        //this is our starting right hand random move
        TrajectorySequence moveToRight = drive.trajectorySequenceBuilder(startingPosition)
                .lineToSplineHeading(new Pose2d(61,-25.5, Math.toRadians(180)))
                .build();

        TrajectorySequence moveToPixelLeft = drive.trajectorySequenceBuilder(moveToLeft.end())
                .lineToSplineHeading(new Pose2d(20,-18, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(18,-22, Math.toRadians(180)),
                        BotBuildersMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        TrajectorySequence moveToPixelRight = drive.trajectorySequenceBuilder(moveToRight.end())
                .lineToSplineHeading(new Pose2d(40,-20, Math.toRadians(180)))
                .build();

        TrajectorySequence moveIntoCenterPosition = drive.trajectorySequenceBuilder(movePixelToCenter.end())
                .lineToSplineHeading(new Pose2d(38, 1, Math.toRadians(185))) //DON"T CHANGE
                .build();

        TrajectorySequence moveLeftIntoMovePos = drive.trajectorySequenceBuilder(moveToPixelLeft.end())
                .lineToSplineHeading(new Pose2d(38, 1, Math.toRadians(185))) //DON'T CHANGE
                .build();

        TrajectorySequence moveRightIntoMovePos = drive.trajectorySequenceBuilder(moveToPixelRight.end())
                .lineToSplineHeading(new Pose2d(50,-16, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(38, 1, Math.toRadians(185))) //DON"T CHANGE
                .build();



        //move down to the stack for the center pos, drive slowly on top of it.
        TrajectorySequence moveCenterPath = drive.trajectorySequenceBuilder(moveIntoCenterPosition.end())

                .lineToSplineHeading(new Pose2d(-45, -11.5, Math.toRadians(185)))
                .lineToSplineHeading(new Pose2d(-56,-11.5, Math.toRadians(185)),
                        BotBuildersMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-48.25, -11.5, Math.toRadians(185)),
                        BotBuildersMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //drive back towards the stack slowly, then move back again.
        TrajectorySequence moveOffStackTraj = drive.trajectorySequenceBuilder(moveCenterPath.end())
                .lineToSplineHeading(new Pose2d(-45, -11.5, Math.toRadians(185)))
                .build();

        //race up to the other side of the stage door
        TrajectorySequence backToBackDrop = drive.trajectorySequenceBuilder(moveOffStackTraj.end())
                .lineToSplineHeading(new Pose2d(35, -4, Math.toRadians(185)))
                .build();

        //get into position, still off the backdrop - ready to deploy the arm
        TrajectorySequence backToBackDrop2 = drive.trajectorySequenceBuilder(backToBackDrop.end())
                .lineToSplineHeading(new Pose2d(55, -4, Math.toRadians(185)))
                .build();

        //side game now deployed, move onto the back drop
        TrajectorySequence moveBackWithSideGame = drive.trajectorySequenceBuilder(backToBackDrop2.end())
                .lineToSplineHeading(new Pose2d(59, -4, Math.toRadians(185)))
                .build();

        TrajectorySequence moveOffBackdrop = drive.trajectorySequenceBuilder(moveBackWithSideGame.end())

                .lineToSplineHeading(new Pose2d(55, -4, Math.toRadians(185)))
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


        TrajectorySequenceFollowerCommand moveToPathCenterFollower = new TrajectorySequenceFollowerCommand(drive, moveCenterPath);
        TrajectorySequenceFollowerCommand moveToPathRightFollower = new TrajectorySequenceFollowerCommand(drive, moveCenterPath);
        TrajectorySequenceFollowerCommand moveToPathLeftFollower = new TrajectorySequenceFollowerCommand(drive, moveCenterPath);

        TrajectorySequenceFollowerCommand moveOffStackFollower = new TrajectorySequenceFollowerCommand(drive, moveOffStackTraj);
        TrajectorySequenceFollowerCommand moveOffStackLeftFollower = new TrajectorySequenceFollowerCommand(drive, moveOffStackTraj);
        TrajectorySequenceFollowerCommand moveOffStackRightFollower = new TrajectorySequenceFollowerCommand(drive, moveOffStackTraj);

        TrajectorySequenceFollowerCommand moveBackToBackdropFollower = new TrajectorySequenceFollowerCommand(drive, backToBackDrop);
        TrajectorySequenceFollowerCommand moveBackToBackdropLeftFollower = new TrajectorySequenceFollowerCommand(drive, backToBackDrop);
        TrajectorySequenceFollowerCommand moveBackToBackdropRightFollower = new TrajectorySequenceFollowerCommand(drive, backToBackDrop);


        TrajectorySequenceFollowerCommand moveBackToBackdropFollower2 = new TrajectorySequenceFollowerCommand(drive, backToBackDrop2);
        TrajectorySequenceFollowerCommand moveBackToBackdropLeftFollower2 = new TrajectorySequenceFollowerCommand(drive, backToBackDrop2);
        TrajectorySequenceFollowerCommand moveBackToBackdropRightFollower2 = new TrajectorySequenceFollowerCommand(drive, backToBackDrop2);


        TrajectorySequenceFollowerCommand backwithSideGameFollower = new TrajectorySequenceFollowerCommand(drive, moveBackWithSideGame);
        TrajectorySequenceFollowerCommand backwithSideGameLeftFollower = new TrajectorySequenceFollowerCommand(drive, moveBackWithSideGame);
        TrajectorySequenceFollowerCommand backwithSideGameRightFollower = new TrajectorySequenceFollowerCommand(drive, moveBackWithSideGame);


        TrajectorySequenceFollowerCommand moveOffSideGame = new TrajectorySequenceFollowerCommand(drive, moveOffBackdrop);
        TrajectorySequenceFollowerCommand moveOffSideGameLeft = new TrajectorySequenceFollowerCommand(drive, moveOffBackdrop);
        TrajectorySequenceFollowerCommand moveOffSideGameRight = new TrajectorySequenceFollowerCommand(drive, moveOffBackdrop);



        intakeSubsystem.ExtendPurple();

        //wait for the op mode to start, then execute our paths.

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
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
                                        new WaitCommand(300),
                                        new DropPixelCommand(armSubsystem),
                                        new WaitCommand(250),
                                        new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                        new WaitCommand(500),
                                        movePixelToLeftFollower,
                                        new RetractPurpleCommand(intakeSubsystem),
                                        moveToLeftFollower,
                                        moveToPathLeftFollower,
                                        new LockTransferCommand(armSubsystem),
                                        //we have now driven onto and off the stack
                                        // turn the intake on, give it a second
                                        //then keep the intake on and move forward slowly
                                        new SequentialCommandGroup(
                                                //this is where we
                                                new IntakeOnCommand(intakeSubsystem),
                                                new WaitCommand(1000),
                                                moveOffStackLeftFollower,
                                                new WaitCommand(500),
                                                new ParallelCommandGroup(
                                                        moveBackToBackdropLeftFollower,
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(1800),
                                                                new IntakeOffCommand(intakeSubsystem)
                                                        )
                                                ),
                                                new UnlockTransferCommand(armSubsystem),

                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new MiddleArmUpCommand(armSubsystem),
                                                                new Pos1ExtendCommand(verticalSubsystem)
                                                        ),

                                                        moveBackToBackdropLeftFollower2
                                                ),
                                                new ArmUpLeftAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(500),
                                                backwithSideGameLeftFollower,
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(800),
                                                moveOffSideGameLeft,
                                                new ArmDownAuto(armSubsystem, verticalSubsystem, state)


                                        )
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
                                            new WaitCommand(300),
                                            new DropPixelCommand(armSubsystem),
                                            new WaitCommand(250),
                                            new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                            new WaitCommand(500),
                                            movePixelToRightFollower,
                                            new RetractPurpleCommand(intakeSubsystem),
                                            moveToRightFollower,
                                            moveToPathRightFollower,
                                                new LockTransferCommand(armSubsystem),
                                                //we have now driven onto and off the stack
                                                // turn the intake on, give it a second
                                                //then keep the intake on and move forward slowly
                                                new SequentialCommandGroup(
                                                        //this is where we
                                                        new IntakeOnCommand(intakeSubsystem),
                                                        new WaitCommand(1000),
                                                        moveOffStackRightFollower,
                                                        new WaitCommand(500),
                                                        new ParallelCommandGroup(
                                                                moveBackToBackdropRightFollower,
                                                                new SequentialCommandGroup(
                                                                        new WaitCommand(1800),
                                                                        new IntakeOffCommand(intakeSubsystem)
                                                                )
                                                        ),
                                                        new UnlockTransferCommand(armSubsystem),

                                                        new ParallelCommandGroup(
                                                                new SequentialCommandGroup(
                                                                        new MiddleArmUpCommand(armSubsystem),
                                                                        new Pos1ExtendCommand(verticalSubsystem)
                                                                ),

                                                                moveBackToBackdropRightFollower2
                                                        ),
                                                        new ArmUpLeftAuto(armSubsystem, verticalSubsystem, state),
                                                        new WaitCommand(500),
                                                        backwithSideGameRightFollower,
                                                        new DropPixelCommand(armSubsystem),
                                                        new WaitCommand(800),
                                                        moveOffSideGameRight,
                                                        new ArmDownAuto(armSubsystem, verticalSubsystem, state)


                                                )
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
                                                new WaitCommand(300),
                                                new DropPixelCommand(armSubsystem),
                                                new WaitCommand(250),
                                                new ArmDownAuto(armSubsystem, verticalSubsystem, state),
                                                new WaitCommand(500),
                                                movePixelToCenterFollower,
                                                new RetractPurpleCommand(intakeSubsystem),
                                                moveToCenterFollower,
                                                moveToPathCenterFollower,
                                                new LockTransferCommand(armSubsystem),
                                                //we have now driven onto and off the stack
                                                // turn the intake on, give it a second
                                                //then keep the intake on and move forward slowly
                                                new SequentialCommandGroup(
                                                        //this is where we
                                                        new IntakeOnCommand(intakeSubsystem),
                                                        new WaitCommand(1000),
                                                        moveOffStackFollower,
                                                        new WaitCommand(500),
                                                        new ParallelCommandGroup(
                                                                moveBackToBackdropFollower,
                                                                new SequentialCommandGroup(
                                                                        new WaitCommand(1800),
                                                                        new IntakeOffCommand(intakeSubsystem)
                                                                )
                                                        ),
                                                        new UnlockTransferCommand(armSubsystem),

                                                        new ParallelCommandGroup(
                                                                new SequentialCommandGroup(
                                                                        new MiddleArmUpCommand(armSubsystem),
                                                                        new Pos1ExtendCommand(verticalSubsystem)
                                                                ),
                                                                moveBackToBackdropFollower2
                                                        ),
                                                        new ArmUpLeftAuto(armSubsystem, verticalSubsystem, state),
                                                        new WaitCommand(500),
                                                        backwithSideGameFollower,
                                                        new DropPixelCommand(armSubsystem),
                                                        new WaitCommand(800),
                                                        moveOffSideGame,
                                                        new ArmDownAuto(armSubsystem, verticalSubsystem, state)


                                                )


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
