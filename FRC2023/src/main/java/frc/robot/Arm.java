package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;

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
        SHELF,
        MID_CONE,
        FLOOR,
        REST
    }

    private ArmMode armMode = ArmMode.IDLE;

    // motors
    private CANSparkMax elbowMotor;
    private CANSparkMax elbowMotor2;
    private CANSparkMax shoulderMotor;
    private CANSparkMax grabberMotor;

    private SparkMaxPIDController shoulderPIDController;
    private SparkMaxPIDController elbowPIDController;
    private SparkMaxPIDController elbowPIDController2;

    private static final double GRABBER_HIGH = .5;

    private double grabberSpeed = 0.1;

    private double maxOutput = 1;
    private double minOutput = -1;
    private double shoulderSetPoint = 0;
    private double elbowSetPoint2 = 0;
    private double shoulderTargetSetPoint = 0;
    private double elbowTargetSetPoint = 0;
    private double shoulderTargetSpeed = .001;
    private double elbowTargetSpeed = .001;
    private double grabberTargetPosition = 0;
    private double shoulderLimitPos = 12;
    private double shoulderLimitNeg = 0;

    private boolean shoulder = false;
    private boolean elbow = false;
    private boolean grabber = false;

    public Arm() {

        elbowMotor = new CANSparkMax(PortMap.CAN.ELBOW_MOTOR, MotorType.kBrushless);
        elbowMotor2 = new CANSparkMax(PortMap.CAN.ELBOW_MOTOR_TWO, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(PortMap.CAN.SHOULDER_MOTOR, MotorType.kBrushless);
        grabberMotor = new CANSparkMax(PortMap.CAN.GRABBER_MOTOR, MotorType.kBrushless);

        elbowMotor.restoreFactoryDefaults();
        elbowMotor2.restoreFactoryDefaults();
        shoulderMotor.restoreFactoryDefaults();
        grabberMotor.restoreFactoryDefaults();

        elbowMotor.setInverted(false);
        elbowMotor2.setInverted(false);
        shoulderMotor.setInverted(false);
        grabberMotor.setInverted(false);

        shoulderMotor.setInverted(false);
        shoulderPIDController = shoulderMotor.getPIDController();
        shoulderPIDController.setOutputRange(minOutput, maxOutput);
        shoulderPIDController.setP(.5);
        shoulderPIDController.setI(0);
        shoulderPIDController.setD(7);
        shoulderPIDController.setFF(0.0);

        elbowPIDController = elbowMotor.getPIDController();
        elbowPIDController.setOutputRange(minOutput, maxOutput);
        elbowPIDController.setP(2);
        elbowPIDController.setI(0);
        elbowPIDController.setD(0.8);
        elbowPIDController.setFF(0.05);

        elbowPIDController2 = elbowMotor2.getPIDController();
        elbowPIDController2.setOutputRange(minOutput, maxOutput);
        elbowPIDController2.setP(4);
        elbowPIDController2.setI(0);
        elbowPIDController2.setD(0.8);
        elbowPIDController2.setFF(0.05);


    }

    @Override
    public void init() {
        stop();
        resetPosition();
        shoulderTargetSetPoint = 0;
        elbowTargetSetPoint = 0;
        grabberTargetPosition = 0;
        // shoulderMotor.restoreFactoryDefaults();
        armMode = ArmMode.IDLE;

    }

    @Override
    public void stop() {
        grabberSpeed = 0.0;
    }

    @Override
    public void resetPosition() {
        shoulderMotor.getEncoder().setPosition(0);
        elbowMotor.getEncoder().setPosition(0); 
        elbowMotor2.getEncoder().setPosition(0); 
        grabberMotor.getEncoder().setPosition(0);
        
        shoulderSetPoint = 0;
        elbowSetPoint2 = 0;
        shoulderTargetSetPoint = 0;
        elbowTargetSetPoint = 0;
    }

    @Override
    public void autoShoulder(double shoulderPosition,double speed, boolean shoulderBoolean, Runnable completionRoutine) {
        mode = Mode.AUTO;
        shoulderTargetSetPoint = shoulderPosition;
        shoulderTargetSpeed = speed;
        shoulder = shoulderBoolean;

        setCompletionRoutine(completionRoutine);

    }

    @Override
    public void autoElbow(double elbowPosition, double speed, boolean elbowBoolean, Runnable completionRoutine) {
        mode = Mode.AUTO;
        elbowTargetSetPoint = elbowPosition;
        elbowTargetSpeed = speed;
        elbow = elbowBoolean;

        setCompletionRoutine(completionRoutine);
    }

    @Override
    public void autoGrabber(double grabberPosition, double speed, boolean grabberBoolean, Runnable completionRoutine) {
        mode = Mode.AUTO;
        grabberTargetPosition = grabberPosition;
        grabberSpeed = speed;
        grabber = grabberBoolean;

        setCompletionRoutine(completionRoutine);
    }

    @Override 
    public void pickUp() {
        // teleop arm positions
        mode = Mode.MANUAL;
        armMode = ArmMode.FLOOR;
    }
    @Override 
    public void midCone() {
        mode = Mode.MANUAL;
        armMode = ArmMode.MID_CONE;
    }
    @Override
    public void shelf() {
        mode = Mode.MANUAL;
        armMode = ArmMode.SHELF;
    }
    @Override
    public void floor() {
        mode = Mode.MANUAL;
        armMode = ArmMode.FLOOR;
    }

    @Override
    public void rest() {
        mode = Mode.MANUAL;
        armMode = ArmMode.REST;
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

    private void setCompletionRoutine(Runnable completionRoutine) {
        if (currentCompletionRoutine != null) {
            throw new IllegalStateException("Tried to perform an autonomous action while one was already in progress!");
        }

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
        Debug.logPeriodic("Grabber" + Double.toString(grabberMotor.getEncoder().getPosition()));
        stop();
        if (shoulderTargetSetPoint > shoulderLimitPos) {
            shoulderTargetSetPoint = shoulderLimitPos;
        }
        if (shoulderTargetSetPoint < shoulderLimitNeg) {
            shoulderTargetSetPoint = shoulderLimitNeg;
        }
        if (shoulderTargetSetPoint - shoulderSetPoint - .02 > 0) {
            shoulderSetPoint += shoulderTargetSpeed;
        }
        if (shoulderTargetSetPoint - shoulderSetPoint + .02 < 0) {
            shoulderSetPoint -= shoulderTargetSpeed;
        }
        if (elbowTargetSetPoint - elbowSetPoint2 - .01 > 0) {
            elbowSetPoint2 += elbowTargetSpeed;
        }
        if (elbowTargetSetPoint - elbowSetPoint2 + .01 < 0) {
            elbowSetPoint2 -= elbowTargetSpeed;
        }
        if (mode == Mode.MANUAL) {
            if (armMode == ArmMode.SHOULDER_UP) {
                shoulderSetPoint += .00075;
                shoulderTargetSetPoint = shoulderSetPoint;
                armMode = ArmMode.IDLE;
                Debug.logPeriodic("working");
            } else if (armMode == ArmMode.SHOULDER_DOWN){
                shoulderSetPoint -= .00075;
                shoulderTargetSetPoint = shoulderSetPoint;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.ELBOW_UP) {
                elbowSetPoint2 -= .00075;
                elbowTargetSetPoint = elbowSetPoint2;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.ELBOW_DOWN) {
                // elbowSpeed = -ELBOW_HIGH;
                elbowSetPoint2 += .00075;
                elbowTargetSetPoint = elbowSetPoint2;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.GRABBER_OPEN) {
                if (grabberMotor.getEncoder().getPosition() < -50) {
                    grabberSpeed = 0;
                } else grabberSpeed = -GRABBER_HIGH;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.GRABBER_CLOSE) {
                if (grabberMotor.getEncoder().getPosition() > 250) {
                    grabberSpeed = 0;
                } else grabberSpeed = GRABBER_HIGH;
                armMode = ArmMode.IDLE;
            } else if (armMode == ArmMode.MID_CONE) {
                shoulderTargetSetPoint = 2.5;
                elbowTargetSetPoint = 7;
            } else if (armMode == ArmMode.SHELF) {
                shoulderTargetSetPoint = 1.6;
                elbowTargetSetPoint = 6;
            } else if (armMode == ArmMode.FLOOR) {
                shoulderTargetSetPoint = 0;
                elbowTargetSetPoint = 8.5;
            } else if (armMode == ArmMode.REST) {
                shoulderTargetSetPoint = 0;
                elbowTargetSetPoint = 0;
            }


        } else if (mode == Mode.AUTO) {
            double currentShoulderPosition = shoulderMotor.getEncoder().getPosition();
            double currentElbowPosition = elbowMotor2.getEncoder().getPosition();
            double currentGrabberPosition = grabberMotor.getEncoder().getPosition();
            Debug.logPeriodic("Current elbow position: " + Double.toString(Math.abs(currentElbowPosition)));
            Debug.logPeriodic("Current elbow target setpoint: " + Double.toString(Math.abs(elbowTargetSetPoint)));
            if(!shoulder) {
                stop();
            } else {
                if (Math.abs(currentShoulderPosition - shoulderTargetSetPoint) < 0.3) {
                    shoulder = false;
                    handleActionEnd();
                    Debug.logPeriodic("print");
                }
            }
            if(!elbow) {
                stop();
            } else {
                if (Math.abs(currentElbowPosition - elbowTargetSetPoint) < 0.3) {
                    elbow = false;
                    handleActionEnd();
                }
            }

            if(!grabber) {
                stop();
            } else {
                Debug.logPeriodic(Double.toString(Math.abs(currentGrabberPosition - grabberTargetPosition)));
                Debug.logPeriodic("grab target: " + Double.toString(grabberTargetPosition));
                Debug.logPeriodic("grab current: " + Double.toString(currentGrabberPosition));
                if (Math.abs(currentGrabberPosition - grabberTargetPosition) < 5) {
                    grabberSpeed = 0;
                    grabber = false;
                    handleActionEnd();
                }
                else if (currentGrabberPosition > grabberTargetPosition) {
                    grabberSpeed = -GRABBER_HIGH;
                }
                else if (currentGrabberPosition < grabberTargetPosition) {
                    grabberSpeed = GRABBER_HIGH;
                }
                // if (!(Math.abs(currentGrabberPosition - grabberTargetPosition) < 5)) {
                //  Debug.logPeriodic(Double.toString(Math.abs(currentGrabberPosition - grabberTargetPosition)));
                //  Debug.logPeriodic("grab target: " + Double.toString(grabberTargetPosition));
                //  Debug.logPeriodic("grab current: " + Double.toString(currentGrabberPosition));
                //  if (currentGrabberPosition > grabberTargetPosition) {
                //      grabberSpeed = -GRABBER_HIGH;
                //    }
                //  else if (currentGrabberPosition < grabberTargetPosition) {
                //      grabberSpeed = GRABBER_HIGH;
                //   }
                // } else { grabberSpeed = 0; }
            }
        }
        elbowPIDController2.setReference(elbowSetPoint2, CANSparkMax.ControlType.kPosition);
        shoulderPIDController.setReference(shoulderSetPoint , CANSparkMax.ControlType.kPosition);

        grabberMotor.set(grabberSpeed);
        // Debug.logPeriodic("Grabber position: " + Double.toString(grabberMotor.getEncoder().getPosition()));
        // Debug.logPeriodic("Applied Output:" + Double.toString(elbowMotor2.getAppliedOutput()));
        // Debug.logPeriodic("Applied Output:" + Double.toString(shoulderMotor.getAppliedOutput()));
        // Debug.logPeriodic("Shoulder Position: " + Double.toString(shoulderMotor.getEncoder().getPosition()));
        // // Debug.logPeriodic("Elbow Position: " + Double.toString(elbowMotor.getEncoder().getPosition()));
        // Debug.logPeriodic("Elbow Position: " + Double.toString(elbowMotor2.getEncoder().getPosition()));
        // Debug.logPeriodic("Shoulder Set Point: " + Double.toString(shoulderSetPoint));
        // Debug.logPeriodic("Elbow Set Point: " + Double.toString(elbowSetPoint2));        
        // // Debug.logPeriodic("Elbow 2 Set Point: " + Double.toString(elbowSetPoint2));
    }
}



