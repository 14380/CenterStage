package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {

    private DcMotorEx intakeMotor;
    private CRServo intakeServo;

    private DistanceSensor sensorDistanceTop;
    private DistanceSensor sensorDistanceBottom;

    private ElapsedTime timer;

    private double cachedDistance = 0;

    private DigitalChannel intake1;
    private DigitalChannel intake2;

    private Servo intakeGate;

    private Servo purplePixel;

    private RobotStateSubsystem robotState;

    public IntakeSubsystem(HardwareMap map, RobotStateSubsystem state){
        intakeMotor = map.get(DcMotorEx.class, "intake");
        intakeServo = map.get(CRServo.class, "intakeServo");
        intake1 = map.get(DigitalChannel.class, "intake1");
        intake2 = map.get(DigitalChannel.class, "intake2");
        purplePixel = map.get(Servo.class, "purplePixel");

        intakeGate = map.get(Servo.class, "intakeGate");

        purplePixel.setPosition(0);

        intakeGate.setPosition(0);
        intakeGate.setDirection(Servo.Direction.REVERSE);

        //sensorDistanceTop = map.get(DistanceSensor.class, "topC");
        //sensorDistanceBottom = map.get(DistanceSensor.class, "bottomC");
        timer = new ElapsedTime();

        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        intake1.setMode(DigitalChannel.Mode.INPUT);
        intake2.setMode(DigitalChannel.Mode.INPUT);

        robotState = state;

    }

    public boolean IsTopCovered(){

      if(IntakeSensor1() && IntakeSensor2()) {
        return true;
      }
        return false;
    }

    public boolean IsBottomCovered(){
        double distance = sensorDistanceBottom.getDistance(DistanceUnit.MM);

        if(distance <= 15){
            return true;
        }
        return false;
    }

    public boolean isExtended(){
        return robotState.horizontalHeight == RobotStateSubsystem.HorizontalHeight.EXTENDED;
    }

    public double getTopDistance(){
        return sensorDistanceTop.getDistance(DistanceUnit.MM);
    }

    public double getBottomDistance(){
        return sensorDistanceBottom.getDistance(DistanceUnit.MM);
    }
    public void IntakeOn(){
        intakeMotor.setPower(1);
        if(robotState.horizontalHeight == RobotStateSubsystem.HorizontalHeight.EXTENDED){
            intakeGate.setPosition(0.5);
            if(IsTopCovered()){
                IntakeAdvanceSpeed(0);
            }else{
                IntakeAdvanceSpeed(1);
            }
        }else{
            intakeGate.setPosition(0);
            IntakeAdvanceSpeed(1);
        }

    }

    public void IntakeAdvanceSpeed(double speed){
        intakeServo.setPower(speed);
    }

    public void IntakeAdvance(){
        intakeServo.setPower(1);
    }

    public void IntakeOff(){
        intakeMotor.setPower(0);
        intakeServo.setPower(0);
    }

    public void Reverse(){
        intakeMotor.setPower(-1);
        intakeServo.setPower(-1);
    }

    public void ExtendPurple(){
        purplePixel.setPosition(0.85);//0.88);
    }

    public void RetractPurple(){

        purplePixel.setPosition(0);
    }
    public boolean IntakeSensor1(){
        return !intake1.getState();
    }

    public boolean IntakeSensor2(){
        return !intake2.getState();
    }
}
