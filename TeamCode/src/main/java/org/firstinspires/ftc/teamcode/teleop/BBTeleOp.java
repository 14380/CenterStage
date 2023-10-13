package org.firstinspires.ftc.teamcode.teleop;



        import com.acmerobotics.dashboard.FtcDashboard;
        import com.acmerobotics.dashboard.config.Config;
        import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

        import com.arcrobotics.ftclib.command.CommandOpMode;
        import com.arcrobotics.ftclib.command.CommandScheduler;

        import com.arcrobotics.ftclib.command.ParallelCommandGroup;
        import com.arcrobotics.ftclib.gamepad.GamepadEx;
        import com.arcrobotics.ftclib.gamepad.GamepadKeys;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;

        import org.firstinspires.ftc.teamcode.commands.drive.DriveCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeOffCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeOnCommand;
        import org.firstinspires.ftc.teamcode.commands.intake.IntakeReverseCommand;
        import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
        import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
        import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Config
@TeleOp(group = "drive")
public class BBTeleOp extends CommandOpMode {

    private DriveCommand driveCommand;
    private IntakeSubsystem intakeSubsystem;

    private IntakeOnCommand intakeOnCommand;
    private GamepadEx gp1;
    private GamepadEx gp2;
    private BotBuildersMecanumDrive mecDrive;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mecDrive = new BotBuildersMecanumDrive(hardwareMap);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);


        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);




        DriveSubsystem driveSystem = new DriveSubsystem(
                mecDrive, gp1, telemetry);


        driveCommand = new DriveCommand(
                driveSystem, () -> -gp1.getLeftY(),
                gp1::getLeftX, gp1::getRightX
        );


        schedule(driveCommand);

        // update telemetry every loop
        //schedule(new RunCommand(telemetry::update));


    }

    @Override
    public void run(){

        CommandScheduler.getInstance().run();


       gp1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
               //intake on
               new IntakeOnCommand(intakeSubsystem)
       ).whenReleased(
               //intake off.
               new IntakeOffCommand(intakeSubsystem)
       );

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                //intake reversed

                new IntakeReverseCommand(intakeSubsystem)

        ).whenReleased(
                //intake off.
                new IntakeOffCommand(intakeSubsystem)
        );



        if(gp1.isDown(GamepadKeys.Button.X) && gp1.isDown(GamepadKeys.Button.Y)) {
            mecDrive.ReAlignIMU();
        }

    }



}

