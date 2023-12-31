package org.firstinspires.ftc.teamcode.commands.autogroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.arm.ArmExtendoOutCommand;
import org.firstinspires.ftc.teamcode.commands.arm.ArmLeftCommand;
import org.firstinspires.ftc.teamcode.commands.arm.ArmRightCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateTransferLeftCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateTransferRightCommand;
import org.firstinspires.ftc.teamcode.commands.arm.UnlockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.Pos1ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.StagedVerticalCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class ArmUpLeftAuto extends SequentialCommandGroup {

    public ArmUpLeftAuto(ArmSubsystem arm, VerticalSlideSubsystem vertSlide, RobotStateSubsystem state){

        addCommands(

                new UnlockTransferCommand(arm),
                new MiddleArmUpCommand(arm),
                new InstantCommand(()->{
                    state.middleArm = RobotStateSubsystem.MiddleArmState.UP;
                }),
                new Pos1ExtendCommand(vertSlide),
                new ArmRightCommand(arm),
                new ArmExtendoOutCommand(arm),
                new RotateTransferRightCommand(arm)
        );

        addRequirements(arm, state);
    }
}
