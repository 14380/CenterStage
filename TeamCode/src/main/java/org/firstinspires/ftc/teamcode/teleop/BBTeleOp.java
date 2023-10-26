package org.firstinspires.ftc.teamcode.teleop;



        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

        import com.arcrobotics.ftclib.command.CommandOpMode;
        import com.arcrobotics.ftclib.command.CommandScheduler;

        import com.arcrobotics.ftclib.command.ConditionalCommand;
        import com.arcrobotics.ftclib.command.InstantCommand;
        import com.arcrobotics.ftclib.command.ParallelCommandGroup;
        import com.arcrobotics.ftclib.command.SequentialCommandGroup;
        import com.arcrobotics.ftclib.gamepad.GamepadEx;
        import com.arcrobotics.ftclib.gamepad.GamepadKeys;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode.commands.arm.ArmExtendoOutCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.ArmLeftCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.ArmRightCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.DropPixelCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmCenterCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmDownCommand;
        import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
        import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.ExtendHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.IncExtendHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.IncOffHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.horizontal.RetractHorizontalCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeAdvanceCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeReverseCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.ExtendVerticalCommand;
        import org.firstinspires.ftc.teamcode.commands.vertical.RetractVerticalCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchOffCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchOnCommand;
        import org.firstinspires.ftc.teamcode.commands.winch.WinchReverseCommand;
        import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
        import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlideSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.WinchSubsystem;

@Config
@TeleOp(group = "drive")
public class BBTeleOp extends CommandOpMode {

    private DriveCommand driveCommand;
    private IntakeSubsystem intakeSubsystem;

    private HorizontalSlideSubsystem horizontalSlideSubsystem;

    private VerticalSlideSubsystem verticalSlideSubsystem;

    private WinchSubsystem winchSubsystem;

    private ArmSubsystem armSubsystem;

    private RobotStateSubsystem stateSubsystem;

    private IntakeOnCommand intakeOnCommand;
    private GamepadEx gp1;
    private GamepadEx gp2;
    private BotBuildersMecanumDrive mecDrive;

    public static double transferSetpoint = 2.55;
    public static double armSetpoint = 1.8;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mecDrive = new BotBuildersMecanumDrive(hardwareMap);

        stateSubsystem = new RobotStateSubsystem();

        intakeSubsystem = new IntakeSubsystem(hardwareMap, stateSubsystem);
        horizontalSlideSubsystem = new HorizontalSlideSubsystem(hardwareMap, stateSubsystem);
        winchSubsystem = new WinchSubsystem(hardwareMap, stateSubsystem);
        verticalSlideSubsystem = new VerticalSlideSubsystem(hardwareMap, stateSubsystem);
        armSubsystem = new ArmSubsystem(hardwareMap, stateSubsystem);



        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);




        DriveSubsystem driveSystem = new DriveSubsystem(
                mecDrive, gp1, telemetry);


        driveCommand = new DriveCommand(
                driveSystem, () -> -gp1.getLeftY(),
                gp1::getLeftX, gp1::getRightX
        );


        schedule(driveCommand);



    }

    @Override
    public void run(){

        armSubsystem.TransferAngle(transferSetpoint);
        armSubsystem.ArmAngle(armSetpoint);


        //call loop on the arm subsystem, this keeps the PID's running
        armSubsystem.loop();
        //schedule the run after we update our loop.
        CommandScheduler.getInstance().run();

       gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
               //intake on
               new IntakeOnCommand(intakeSubsystem)
       ).whenReleased(
               //intake off.
               new IntakeOffCommand(intakeSubsystem)
       );

        gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                //intake reversed

                new IntakeReverseCommand(intakeSubsystem)

        ).whenReleased(
                //intake off.
                new IntakeOffCommand(intakeSubsystem)
        );

        //conditional transfer, or extendo when the arm is up
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(

                 new ConditionalCommand(
                         new IntakeAdvanceCommand(intakeSubsystem),
                         new ArmExtendoOutCommand(armSubsystem),
                    () ->{
                        return stateSubsystem.middleArm == RobotStateSubsystem.MiddleArmState.UP;
                    }
                )
        );

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                    new IncExtendHorizontalCommand(horizontalSlideSubsystem),
                    new InstantCommand( () -> {
                        stateSubsystem.horizontalHeight = RobotStateSubsystem.HorizontalHeight.EXTENDED;
                    })
                )
        ).whenReleased(
                new SequentialCommandGroup(
                    new IncOffHorizontalCommand(horizontalSlideSubsystem),
                        new InstantCommand( () -> {
                            stateSubsystem.horizontalHeight = RobotStateSubsystem.HorizontalHeight.EXTENDED;
                        })
                )
        );

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(

            new SequentialCommandGroup(
                    new RetractHorizontalCommand(horizontalSlideSubsystem),
                    new InstantCommand( () -> {
                        stateSubsystem.horizontalHeight = RobotStateSubsystem.HorizontalHeight.RETRACTED;
                    })
            )
        );

        //TODO: conditional - need to keep track of the state and cycle between the states
        //TODO: i.e. first press lifts arm, second moves slides to Pos 1, third moves to Pos 2, forth to Pos1 etc
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(

                new SequentialCommandGroup(
                    new MiddleArmUpCommand(armSubsystem),
                    new InstantCommand(() ->{
                        stateSubsystem.middleArm = RobotStateSubsystem.MiddleArmState.UP;
                    })
                )
        );

        //Collapse the whole arm system depending on state.
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new SequentialCommandGroup(
                new MiddleArmCenterCommand(armSubsystem),
                        new ParallelCommandGroup(
                                new MiddleArmDownCommand(armSubsystem),
                                new RetractVerticalCommand(verticalSlideSubsystem)
                        ),
                        new InstantCommand(() ->{
                            stateSubsystem.middleArm = RobotStateSubsystem.MiddleArmState.DOWN;
                            stateSubsystem.verticalHeight = RobotStateSubsystem.VerticalHeight.DOWN;
                        })
                )

        );

        //only work if the arm is up
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ConditionalCommand(
                    new ArmRightCommand(armSubsystem),
                        new InstantCommand(), //DO nothing
                        () ->{
                        return stateSubsystem.middleArm == RobotStateSubsystem.MiddleArmState.UP;
                        })
        );

        // only work if the arm is up
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(

            new ConditionalCommand(
                    new ArmLeftCommand(armSubsystem),
                    new InstantCommand(), //DO nothing
                    () ->{
                        return stateSubsystem.middleArm == RobotStateSubsystem.MiddleArmState.UP;
                    })
        );


        //only work if the robot is in the delivery state
        gp1.getGamepadButton(GamepadKeys.Button.X).whenPressed(

            new ConditionalCommand(
                new DropPixelCommand(armSubsystem),
                new InstantCommand(), //DO nothing
                () ->{
                    return stateSubsystem.middleArm == RobotStateSubsystem.MiddleArmState.UP;
                })
        );


        schedule(new InstantCommand(() -> telemetry.addData( "Pos", horizontalSlideSubsystem.getCurrentPosition() )));
        schedule(new InstantCommand(() -> telemetry.addData( "Vert", verticalSlideSubsystem.getCurrentPosition() )));

        schedule(new InstantCommand(() -> telemetry.addData( "Transfer", armSubsystem.getTransferAInput() )));

        schedule(new InstantCommand(() -> telemetry.addData( "Arm", armSubsystem.getArmAngle() )));


        //schedule(new InstantCommand(() -> telemetry.addData( "Top", intakeSubsystem.getTopDistance() )));
        //schedule(new InstantCommand(() -> telemetry.addData( "Bottom", intakeSubsystem.getBottomDistance() )));

        schedule(new InstantCommand(()-> telemetry.update()));

        //re-align the IMU - will this work with Odo Wheels?
        if(gp1.isDown(GamepadKeys.Button.X) && gp1.isDown(GamepadKeys.Button.Y)) {
            mecDrive.ReAlignIMU();
        }

    }





}

