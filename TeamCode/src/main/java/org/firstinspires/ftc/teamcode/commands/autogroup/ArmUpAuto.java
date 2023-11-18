package org.firstinspires.ftc.teamcode.commands.autogroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.arm.ArmExtendoOutCommand;
import org.firstinspires.ftc.teamcode.commands.arm.ArmLeftCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateTransferLeftCommand;
import org.firstinspires.ftc.teamcode.commands.arm.UnlockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.StagedVerticalCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class ArmUpAuto extends SequentialCommandGroup {

    public ArmUpAuto(ArmSubsystem arm, VerticalSlideSubsystem vertSlide, RobotStateSubsystem state){

        addCommands(

                new UnlockTransferCommand(arm),
                new MiddleArmUpCommand(arm),
                new InstantCommand(()->{
                    state.middleArm = RobotStateSubsystem.MiddleArmState.UP;
                }) //,

                //new ArmExtendoOutCommand(arm) // STATES: Changed to no extendo

        );

        addRequirements(arm, state);
    }
}
