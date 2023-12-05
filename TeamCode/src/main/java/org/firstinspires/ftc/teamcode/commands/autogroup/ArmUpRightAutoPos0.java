package org.firstinspires.ftc.teamcode.commands.autogroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.arm.ArmExtendoOutCommand;
import org.firstinspires.ftc.teamcode.commands.arm.ArmLeftCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateTransferLeftCommand;
import org.firstinspires.ftc.teamcode.commands.arm.UnlockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.Pos0ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.Pos1ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.PosAExtendCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class ArmUpRightAutoPos0 extends SequentialCommandGroup {

    public ArmUpRightAutoPos0(ArmSubsystem arm, VerticalSlideSubsystem vertSlide, RobotStateSubsystem state){

        addCommands(

                new UnlockTransferCommand(arm),
                new MiddleArmUpCommand(arm),
                new InstantCommand(()->{
                    state.middleArm = RobotStateSubsystem.MiddleArmState.UP;
                }),

                new PosAExtendCommand(vertSlide),
                new ArmLeftCommand(arm),
                new ArmExtendoOutCommand(arm),
                new RotateTransferLeftCommand(arm)

        );

        addRequirements(arm, state);
    }
}
