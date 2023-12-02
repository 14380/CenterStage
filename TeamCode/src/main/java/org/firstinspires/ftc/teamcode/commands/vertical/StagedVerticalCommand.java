package org.firstinspires.ftc.teamcode.commands.vertical;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.RobotStateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlideSubsystem;

public class StagedVerticalCommand extends CommandBase {

    private VerticalSlideSubsystem slideSubsystem;

    private RobotStateSubsystem state;

    public StagedVerticalCommand(VerticalSlideSubsystem slide, RobotStateSubsystem rState) {
        this.slideSubsystem = slide;
        this.state = rState;
    }

    @Override
    public void initialize() {

        if(this.state.middleArm == RobotStateSubsystem.MiddleArmState.UP && this.state.verticalHeight == RobotStateSubsystem.VerticalHeight.DOWN){

            this.slideSubsystem.Position0();

        }
        else if(this.state.middleArm == RobotStateSubsystem.MiddleArmState.UP && this.state.verticalHeight == RobotStateSubsystem.VerticalHeight.POS0){

            this.slideSubsystem.Position1();
        }
        else if(this.state.middleArm == RobotStateSubsystem.MiddleArmState.UP && this.state.verticalHeight == RobotStateSubsystem.VerticalHeight.POS1){

            this.slideSubsystem.Position2();
        }
        else if(this.state.middleArm == RobotStateSubsystem.MiddleArmState.UP && this.state.verticalHeight == RobotStateSubsystem.VerticalHeight.POS2){

            this.slideSubsystem.Position1();
        }
    }

    @Override
    public boolean isFinished()
    {
        if(this.state.middleArm == RobotStateSubsystem.MiddleArmState.UP)
        {
            if(this.state.verticalHeight == RobotStateSubsystem.VerticalHeight.DOWN)
            {

                boolean pos0Result = this.slideSubsystem.IsPosition0();
                if (pos0Result) {
                    this.state.verticalHeight = RobotStateSubsystem.VerticalHeight.POS0;
                }

                return pos0Result;
            }
            else if(this.state.verticalHeight == RobotStateSubsystem.VerticalHeight.POS0){

                boolean pos1Result = this.slideSubsystem.IsPosition1();
                if (pos1Result) {
                    this.state.verticalHeight = RobotStateSubsystem.VerticalHeight.POS1;
                }

                return pos1Result;
            }
            else if(this.state.verticalHeight == RobotStateSubsystem.VerticalHeight.POS1){

                boolean pos2Result = this.slideSubsystem.IsPosition2();
                if (pos2Result) {
                    this.state.verticalHeight = RobotStateSubsystem.VerticalHeight.POS2;
                }

                return pos2Result;
            }
            else if(this.state.verticalHeight == RobotStateSubsystem.VerticalHeight.POS2){

                boolean result = this.slideSubsystem.HasMoveFrom1to2Completed();
                if (result) {
                    this.state.verticalHeight = RobotStateSubsystem.VerticalHeight.POS1;
                }

                return result;
            }

        }
        //need to work out what value
        return true;
    }
}
