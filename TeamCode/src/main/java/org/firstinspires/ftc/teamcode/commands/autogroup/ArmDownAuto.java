package org.firstinspires.ftc.teamcode.commands.autogroup;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.arm.ArmExtendoInCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmDownCommand;
import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmDownSlow1Command;
import org.firstinspires.ftc.teamcode.commands.arm.MiddleArmUpCommand;
import org.firstinspires.ftc.teamcode.commands.arm.PixelCloseCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateCenterCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateTransferCenterCommand;
import org.firstinspires.ftc.teamcode.commands.arm.UnlockTransferCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.RetractVerticalCommand;
import org.firstinspires.ftc.teamcode.commands.vertical.StagedVerticalCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class ArmDownAuto extends SequentialCommandGroup {

    public ArmDownAuto(ArmSubsystem arm, VerticalSlideSubsystem vertSlide, RobotStateSubsystem state){

        addCommands(



                        new RotateTransferCenterCommand(arm),
                        new WaitCommand(50),
                        new RotateCenterCommand(arm),
                        new WaitCommand(50),
                        new ArmExtendoInCommand(arm),
                        new WaitCommand(100),
                        new RetractVerticalCommand(vertSlide, state),
                        new MiddleArmDownSlow1Command(arm),
                        new WaitCommand(50),
                        new MiddleArmDownCommand(arm),
                        new PixelCloseCommand(arm),

                        new InstantCommand(() ->{
                            state.middleArm = RobotStateSubsystem.MiddleArmState.DOWN;
                            state.verticalHeight = RobotStateSubsystem.VerticalHeight.DOWN;

                        })


        );

        addRequirements(arm, state);
    }
}
