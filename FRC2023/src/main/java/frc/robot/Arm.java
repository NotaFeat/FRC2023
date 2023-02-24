package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm implements IArm {

    private Mode mode;
    private Runnable currentCompletionRoutine;

    private enum ArmMode {
        IDLE,
        SHOULDER_UP,
        SHOULDER_DOWN,
        ELBOW_UP,
        ELBOW_DOWN,
        GRABBER_CLOSE,
        GRABBER_OPEN,
    }

    private ArmMode armMode = ArmMode.IDLE;

    // motors
    private CANSparkMax elbowMotor;
    private CANSparkMax shoulderMotor;
    private CANSparkMax grabberMotor;

    // speeds
    private static final double SHOULDER_HIGH = 0.1;
    private static final double ELBOW_HIGH = 0.1;
    private static final double GRABBER_HIGH = 0.1;

    private static final double AUTO_SHOULDER_ROTATIONS = 1; // TODO: Change rotation values
    private static final double AUTO_ELBOW_ROTATIONS = 1;
    private static final double AUTO_GRABBER_ROTATIONS = 1;

    private double maxRPM = 100;

    private double shoulderSpeed = 0.0;
    private double elbowSpeed = 0.0;
    private double grabberSpeed = 0.0;

    public Arm() {

        elbowMotor = new CANSparkMax(PortMap.CAN.ELBOW_MOTOR, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(PortMap.CAN.SHOULDER_MOTOR, MotorType.kBrushless);
        grabberMotor = new CANSparkMax(PortMap.CAN.GRABBER_MOTOR, MotorType.kBrushless);

        elbowMotor.restoreFactoryDefaults();
        shoulderMotor.restoreFactoryDefaults();
        grabberMotor.restoreFactoryDefaults();

        elbowMotor.setInverted(false);
        shoulderMotor.setInverted(false);
        grabberMotor.setInverted(false);

    }

    @Override
    public void init() {
        stop();
        elbowMotor.restoreFactoryDefaults();
        shoulderMotor.restoreFactoryDefaults();
        grabberMotor.restoreFactoryDefaults();
        armMode = ArmMode.IDLE;

    }

    @Override
    public void stop() {
        shoulderSpeed = 0.0;
        elbowSpeed = 0.0;
        grabberSpeed = 0.0;
    }


    @Override
    public void shoulderUp() {
        mode = Mode.MANUAL;
        armMode = ArmMode.SHOULDER_UP;
    }

    @Override
    public void shoulderDown() {
        mode = Mode.MANUAL;
        armMode = ArmMode.SHOULDER_DOWN;
    }

    @Override
    public void elbowUp() {
        mode = Mode.MANUAL;
        armMode = ArmMode.ELBOW_UP;
    }

    @Override
    public void elbowDown() {
        mode = Mode.MANUAL;
        armMode = ArmMode.ELBOW_DOWN;
    }

    @Override
    public void grabberClose() {
        mode = Mode.MANUAL;
        armMode = ArmMode.GRABBER_CLOSE;
    }

    @Override
    public void grabberOpen() {
        mode = Mode.MANUAL;
        armMode = ArmMode.GRABBER_OPEN;
    }

    @Override
    public void autoArm(Runnable completionRoutine) {
        mode = Mode.AUTO;
        shoulderMotor.getEncoder().setPosition(0.0);
        elbowMotor.getEncoder().setPosition(0.0);
        grabberMotor.getEncoder().setPosition(0.0);
        currentCompletionRoutine = completionRoutine;
    }

    private void handleActionEnd() {
        stop();
        
        if (currentCompletionRoutine != null) {
            Runnable oldCompletionRoutine = currentCompletionRoutine;
            currentCompletionRoutine = null;
            oldCompletionRoutine.run();
        }
    }


    @Override
    public void periodic() {
        stop();
        if (mode == Mode.MANUAL) {
            if (armMode == ArmMode.SHOULDER_UP) {
                shoulderSpeed = SHOULDER_HIGH;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.SHOULDER_DOWN){
                shoulderSpeed = -SHOULDER_HIGH;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.ELBOW_UP) {
                elbowSpeed = ELBOW_HIGH;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.ELBOW_DOWN) {
                elbowSpeed = -ELBOW_HIGH;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.GRABBER_OPEN) {
                grabberSpeed = GRABBER_HIGH;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.GRABBER_CLOSE) {
                grabberSpeed = -GRABBER_HIGH;
                armMode = ArmMode.IDLE;
            }

        } else if (mode == Mode.AUTO) {
            // TODO: auto arm
        }
        elbowMotor.set(elbowSpeed);
        shoulderMotor.set(shoulderSpeed);
        grabberMotor.set(grabberSpeed);
    }
}
