#include "drv_piezo.h"
#include "drv_piezo_pwm.h"

#include "FreeRTOS.h"
#include "task.h"
#include "log.h"

#include "stm32f4xx_rcc.h"

#define LOG_MOVEMENT 0

#define CLAMP(x, a, b) (((x) < (a)) ? (a) : (((x) > (b)) ? (b) : (x)))

// Work in progress!
// PZT's PID controller



// State per actuator.
typedef struct PidStateRec
{
	long IirMe5; // IIR_Z11
	long IirMe6; // IIR_X1
	long ErrPro;
	long ErrInt;
	long ErrDif;
} PidState;

typedef struct PidParamRec
{
	long GainPro;
	long GainInt;
	long GainDif;
	long IirF3a; // IIR_K1
	long IirF3b; // IIR_B11
	long IirF3c; // IIR_A10
	long IirF3d; // IIR_A11
	int  m_limit;
} PidParam;

// TODO: Resolve the value for I.  There are four conflicting sources for its value:
// #define	KI	 2768 				// 0.003052 * 2^15 = 100
// long  	g_SlGanInt = KI ;		// 0.003052 * 2^15 = 1100
// defintion of PID gain
// KP+KI+KD=32768 // 2^15=32768

PidParam gPidParam =
{
		 15000, // GanPro = 0.457764 * 2^15 = 15000
		  2768, // GanInt = 0.003052 * 2^15 = 1100
		 15000, // GanDif = 0.457764 * 2^15 = 15000

		 // Lead Lag Filter
		 32767, // IirF3a = F3A = IIR_K1  =  1     * 2^15 =  32767
		 29261, // IirF3b = F3B = IIR_B11 =  0.893 * 2^15 =  29260.931
		 32767, // IirF3c = F3C = IIR_A10 =  1     * 2^15 =  32767
		-32767, // IirF3d = F3D = IIR_A11 = -1     * 2^15 = -32767

		    20, // m_limit = +/- 2% at PID
};

Error_t InitPID(PidState *s);

extern PiezoPwm Pwm;

//==========================================================
// PID routines
//==========================================================
// Initialize the actuator state variables.
Error_t InitPID(PidState *s)
{
	Error_t e = ERROR_NONE;

	ENTER_FUNC;

	CHECK_ERROR(s == NULL ? ERROR_INVALID_ARG : ERROR_NONE);

	s->IirMe5 = 0;
	s->IirMe6 = 0;		// IIR_Z11, IIR_X1
	s->ErrInt = 0;

	EXIT_FUNC;

	return e;
}

/*
 * ComputePIDStep(PidState *s, int err)
 *     Given an error metric, compute the change in duty cycle (currently in %) for the next time interval.
 */
int ComputePIDStep(PidState *s, int err)
{
	// speed control
	// Input:	"err" range: +- 400 that means +-4mm/s error
	// Output:	change of duty by 0.1% unit.
	//static UInt16 cnt = 0;
	int	i;

	// ENTER_FUNC

	err = CLAMP(err, -400, 400);

	// skip moving average

	// Proportional
	s->ErrPro = err;

	// Integral
	s->ErrInt += s->ErrPro;
	s->ErrInt  = CLAMP(s->ErrInt, -1000, 1000);

	// Derivative, IIR
	s->IirMe6  = (gPidParam.IirF3a * s->ErrPro + gPidParam.IirF3b * s->IirMe5) >> 15;
	s->ErrDif  = (gPidParam.IirF3c * s->IirMe6 + gPidParam.IirF3d * s->IirMe5) >> 15;
	s->IirMe5  = s->IirMe6;

	s->ErrDif = CLAMP(s->ErrDif, -400, 400);

	i = (gPidParam.GainPro * s->ErrPro +
	     gPidParam.GainInt * s->ErrInt +
		 gPidParam.GainDif * s->ErrDif) >> 15;

#if 0
	// debug message every 10 times
	if((++cnt % 10) == 0)
	{
		log_debug("err=%d P%d:I%d:D%d FB=%d",
			 err, (int)s->ErrPro, (int)s->ErrInt, (int)s->ErrDif, i);
	}
#endif

	// convert speed to duty.  1mm/s --> 1%
	i /= 10;

	// maximal change is under +-500 which is 5%
	i = CLAMP(i, -gPidParam.m_limit, gPidParam.m_limit);

	return i;
}

// Abort if more than 1000 steps.
#define PIEZO_MOVE_TIMEOUT 200

#ifdef LOG_MOVEMENT
UInt16 MovementHallReadTime  [PIEZO_MOVE_TIMEOUT];
UInt16 MovementPosition      [PIEZO_MOVE_TIMEOUT];
UInt16 MovementTime          [PIEZO_MOVE_TIMEOUT];
UInt16 MovementDTime         [PIEZO_MOVE_TIMEOUT];
UInt16 MovementNewSpeed      [PIEZO_MOVE_TIMEOUT];
UInt16 MovementSpeed         [PIEZO_MOVE_TIMEOUT];
UInt16 MovementTargetSpeed   [PIEZO_MOVE_TIMEOUT];
UInt16 MovementRemaining     [PIEZO_MOVE_TIMEOUT];
Int16  MovementDutyCycleDelta[PIEZO_MOVE_TIMEOUT];
Int16  MovementDutyCycle     [PIEZO_MOVE_TIMEOUT];
#endif

#ifndef SAMPLING
#define SAMPLING 1
#endif

#define ABS(x) (((x) < 0) ? (-(x)) : (x))

// Internal function.  Assumes the Hall sensor is configured, as is the CPLD and PWM.
// All that remains is pulsing the PMW bursts.
// This is intended to be used to tweak the lens position at later stages of algorithm
// optimization.
void NudgeCore(PiezoActuator *a, UInt8 Duration)
{
	// ENTER_FUNC;

	if (!a) return;

	// Turn on PWM for a burst.
	TurnOnPwm (&Pwm, a);

	// Insert delay here.  1ms is way too long - so for now just spin.
	vTaskDelay(Duration); // too long?

	// Turn off PWM to end the burst.
	TurnOffPwm(&Pwm, a);

	// EXIT_FUNC;
}

/*
 * PiezoNudge() - Nudge the actuator in the desired direction with a little burst of PWM.
 */
Error_t PiezoNudge(PiezoActuator *a, UInt8 Direction, UInt16 DutyCycle, UInt8 Duration)
{
	Error_t e = ERROR_NONE;

	// ENTER_FUNC

	// Error checking
	CHECK_ERROR(a == NULL ? ERROR_INVALID_ARG : ERROR_NONE);

	if (!a->CalibData.IsValid)
	{
		// Set a reasonable value for the frequency.
		log_error("Warning: Module not calibrated.  Using a default for PWM frequency");
		a->CalibData.Frequency = 128000;
		a->CalibData.IsValid = 1; // Not really true... but it limits repeat warnings.
	}

	// Set current actuator
	SetCurrentActuator(a);

	// Configure PWM
	ConfigurePwm(&Pwm, a, DutyCycle, Direction, 0);

	NudgeCore(a, Duration);

	log_debug("Nudged position: 0x%04x", ReadHallSensor(&(a->Hall)));

	// EXIT_FUNC

	return e;
}


void Parse32(uint32_t x, int *m, int *t, int *o)
{
	*m = x / 1000000;
	x  = x % 1000000;

	*t = x / 1000;
	*o = x % 1000;
}

/*
 * NOTE: This code's comments refer to positions in um, but
 *       they are really ADC codes at this time.
 * NOTE: The speed assumes regular sampling intervals, which needs further study.
 */
Error_t PiezoMoveToPosition(PiezoActuator *a, UInt16 TargetPosition, UInt16 Tolerance, UInt16 StartingDutyCycle)
{
	UInt16 CurrentPosition;
	UInt16 LastPosition;
	UInt16 StopThreshold;
	UInt16 Counter = PIEZO_MOVE_TIMEOUT;
	UInt8  Direction;
	HallSensor *h;
	Error_t e = ERROR_NONE;
	UInt16 DutyCycle = StartingDutyCycle;
	Int16  RemainingDistance;
	Int16  TargetSpeed;
	Int16  Speed;
	Int16  NewSpeed;
	Int16  DutyCycleDelta;
	Int16  tmp;
	UInt32 Time;
	UInt32 LastTime;
	UInt32 dTime;
	PidState pidState;

	volatile UInt32 t0=0, t1=0;

#if LOG_MOVEMENT
	int i, count = 0;
#endif

	h = &(a->Hall);

	CurrentPosition = ReadHallSensor(h);
	Direction = (TargetPosition > CurrentPosition) ? FAR : NEAR;
	StopThreshold = TargetPosition + ((Direction == NEAR) ? Tolerance : -Tolerance);

	//log_printf("Curr=0x%04x Targ=0x%04x Stop=0x%04x\n", CurrentPosition, TargetPosition, StopThreshold);

	InitPID(&pidState);

	TargetSpeed = 1000; // This was str_setup.speed.
	Speed       = 0; // This is a guess...

	LastPosition = CurrentPosition;

	LastTime = (UInt32) xTaskGetTickCount();

	ConfigurePwm(&Pwm, a, DutyCycle, Direction, 0);
	TurnOnPwm(&Pwm, a);

	while (Counter)
	{
		t0 = (UInt32) xTaskGetTickCount();

		// Determine the current position.
		CurrentPosition = ReadHallSensor(h);
		Time = (UInt32) xTaskGetTickCount();
		t1 = Time;

#if LOG_MOVEMENT
		MovementHallReadTime[count] = t1 - t0;
		MovementPosition    [count] = CurrentPosition;
		MovementTime        [count] = Time;
#endif
		// Check if movement is done.
		// Check if done.
		if (((Direction == FAR ) && (CurrentPosition >= StopThreshold)) ||
			((Direction == NEAR) && (CurrentPosition <= StopThreshold)))
		{
			break;
		}

		// From the current position, compute the remaining distance.
		RemainingDistance = ABS(TargetPosition - CurrentPosition);

		// Adjust the desired speed based on the remaining distance.
		if(RemainingDistance < 100)
		{
			// under 100um
			// if((id%2)==0) // no speed control for mirror-actuator
			tmp = CLAMP(RemainingDistance, 40, 100); // Clamp range to (40%, 100%) of (100um) target speed.
			TargetSpeed = (1000 / 100) * tmp; // The 1000 was str_setup.speed
		}

		// Compute the current speed from the current position + history.
		dTime = (Time == LastTime) ? 1 : (LastTime > Time ? LastTime - Time : Time + (~LastTime + 1));
		NewSpeed = ABS(CurrentPosition - LastPosition)*100/dTime;		// speed by 0.01mm/s (TODO: UNITS!)

		Speed = (Speed + NewSpeed)/2;

		// Determine the change in duty cycle from the PID controller and speed differences.
		DutyCycleDelta = ComputePIDStep(&pidState, TargetSpeed - Speed);	// calc. error

		// Update the duty cycle.
		DutyCycle += DutyCycleDelta;	// if up the duty, up the speed
		DutyCycle  = CLAMP(DutyCycle, DUTY_LO, DUTY_HI);

		ConfigurePwm(&Pwm, a, DutyCycle, Direction, 0);  // Should already be on & running.

#if LOG_MOVEMENT
		MovementDTime         [count] = dTime;
		MovementNewSpeed      [count] = NewSpeed;
		MovementSpeed         [count] = Speed;
		MovementTargetSpeed   [count] = TargetSpeed;
		MovementRemaining     [count] = RemainingDistance;
		MovementDutyCycleDelta[count] = DutyCycleDelta;
		MovementDutyCycle     [count] = DutyCycle;
		count++;
#endif

		// Decrement the timeout counter.
		--Counter;

		LastTime = Time;
		LastPosition = CurrentPosition;
	}


	TurnOffPwm(&Pwm, a);

	CurrentPosition = ReadHallSensor(h);
	log_printf("Target:0x%04x  Final:0x%04x  Iter:0x%04x\n",
				TargetPosition, CurrentPosition, PIEZO_MOVE_TIMEOUT - Counter);

	log_printf("Times: T0:%d  T1:%d\n", (int)t0, (int)t1);

	PrintHallRetryStats();

#if LOG_MOVEMENT
	// Trickle out log data.
	log_printf("c=%d\n", count);
	log_printf("Time,dTime,HallReadTime,Position,Target,Remaining,NewSpeed,Speed,TargetSpeed,DutyCycleDelta,DutyCycle\n");
	for (i = 0; i < count; i++)
	{
		log_printf("%d,", MovementTime[i]);
		log_printf("%d,", MovementDTime[i]);
		log_printf("%d,", MovementHallReadTime[i]);
		log_printf("%d,", MovementPosition[i]);
		log_printf("%d,", TargetPosition);
		log_printf("%d,", MovementRemaining[i]);
		log_printf("%d,", MovementNewSpeed[i]);
		log_printf("%d,", MovementSpeed[i]);
		log_printf("%d,", MovementTargetSpeed[i]);
		log_printf("%d,", MovementDutyCycleDelta[i]);
		log_printf("%d,\n", MovementDutyCycle[i]);

		vTaskDelay(10);
		if ((i % 10) == 0)
		{
			taskYIELD();
		}
	}
#endif

	// TODO: Error reporting.

	return e;
}

// Returns the lens position value in microns.
UInt16 ComputeLensPosition(PiezoActuator *a, UInt32 Distance)
{
    float FocusDistance;
    float Position;

    FocusDistance = Distance;
    log_debug("FocusDistance  = %f mm", FocusDistance);

    // Add the optical path to the mirror.
    // Note: This isn't actually right.  It's the distance from the mirror to the 'thin' lens that matters.
    FocusDistance += a->CalibData.DerivedMirrorDistance;

    //log_debug("FocusDistance' = %f mm", FocusDistance);

    // Calculate the position (mm)
    Position = a->CalibData.DerivedFocalLength * FocusDistance / (FocusDistance - a->CalibData.DerivedFocalLength);

    log_debug("Position       = %2.4f mm", Position);

    // Remove the path from the sensor to the hard stop reference point.
    Position -= a->CalibData.DerivedHardStopOffset;

    log_debug("Position'      = %2.4f mm", Position);

    // Convert to microns and round
    return (1000.0 * Position + 0.5);
}

Error_t PiezoMoveToDistance(PiezoActuator *a, UInt32 Distance)
{
	Error_t e = ERROR_NONE;
	UInt16 TargetPosition;

	ENTER_FUNC

	log_debug("Distance = %d mm", (UInt16)Distance);

	// Compute lens position from distance.
	TargetPosition = ComputeLensPosition(a, Distance);

	log_debug("Target position = %d (0x%04x)", TargetPosition, TargetPosition);

	// Map to Hall sensor - TBD

	// Move the lens to this position.
	// Note: This calls the external interface which ensures that the
	//       hardware is configured properly.
	e = MovePiezoToPosition(a, TargetPosition, 1, 0x90); // TODO: Better starting duty cycle...

	EXIT_FUNC

	return e;
}


#if 0

	/*
	 * NOTE: This code's comments refer to positions in um, but
	 *       they are really ADC codes at this time.
	 * NOTE: The speed assumes regular sampling intervals, which needs further study.
	 */
Error_t PiezoMoveToPosition(PiezoActuator *a, UInt16 TargetPosition)
{
	UInt16 CurrentPosition;
	UInt16 LastPosition;
	UInt16 StopThreshold;
	UInt16 Counter = PIEZO_MOVE_TIMEOUT;
	UInt16 DutyCycle;
	Int16  StopBefore = 5; // TODO: Move this to a configurable parameter.
	Int16  RemainingDistance;
	Int16  TargetSpeed;
	Int16  Speed;
	Int16  NewSpeed;
	Int16  DutyCycleDelta;
	UInt8  Direction;
	Int16  tmp;
	PidState pidState;
	HallSensor *h;
	Error_t e = ERROR_NONE;
#if LOG_MOVEMENT
	int i, count = 0;
#endif

	h = &(a->Hall);

	CurrentPosition = ReadHallSensor(h);
	Direction = (TargetPosition > CurrentPosition) ? FAR : NEAR;
	StopThreshold = TargetPosition + ((Direction == NEAR) ? StopBefore : -StopBefore);

	log_printf("StopThreshold=%d", StopThreshold);

	InitPID(&pidState);

	TargetSpeed = 1000; // This was str_setup.speed.
	Speed       = 0; // This is a guess...
	DutyCycle = DUTY; // Starting duty cycle

	LastPosition = CurrentPosition;

	ConfigurePwm(&Pwm, a, DutyCycle, Direction, 0);
	TurnOnPwm(&Pwm, a);

	while (Counter)
	{
		// Determine the current position.
		CurrentPosition = ReadHallSensor(h);

		// Check if movement is done.
		// Check if done.
		if (((Direction == FAR ) && (CurrentPosition >= StopThreshold)) ||
		    ((Direction == NEAR) && (CurrentPosition <= StopThreshold)))
		{
			break;
		}

		// From the current position, compute the remaining distance.
		RemainingDistance = ABS(TargetPosition - CurrentPosition);

		// Adjust the desired speed based on the remaining distance.
		if(RemainingDistance < 100)
		{
			// under 100um
			// if((id%2)==0) // no speed control for mirror-actuator
			tmp = CLAMP(RemainingDistance, 40, 100); // Clamp range to (40%, 100%) of (100um) target speed.
			TargetSpeed = (1000 / 100) * tmp; // The 1000 was str_setup.speed
		}

		// Compute the current speed from the current position + history.
		NewSpeed = ABS(CurrentPosition - LastPosition)*100/SAMPLING;		// speed by 0.01mm/s

		LastPosition = CurrentPosition;
		Speed = (Speed + NewSpeed)/2;

		// Determine the change in duty cycle from the PID controller and speed differences.
		DutyCycleDelta = ComputePIDStep(&pidState, TargetSpeed - Speed);	// calc. error

		// Update the duty cycle.
		DutyCycle += DutyCycleDelta;	// if up the duty, up the speed
		DutyCycle  = CLAMP(DutyCycle, DUTY_LO, DUTY_HI);

		ConfigurePwm(&Pwm, a, DutyCycle, Direction, 0);  // Should already be on & running.

#if LOG_MOVEMENT
		MovementPosition[count] = CurrentPosition;
		MovementDutyCycle[count] = DutyCycle;
		count++;
#endif

		// Decrement the timeout counter.
		--Counter;
	}

	TurnOffPwm(&Pwm, a);

#if LOG_MOVEMENT
	// Trickle out log data.
	log_printf("c=%d\n", count);
	for (i = 0; i < count; i++)
	{
		log_printf("%d, %d\n", MovementPosition[i], MovementDutyCycle[i]);
		vTaskDelay(10);
		if ((i % 10) == 0)
		{
			taskYIELD();
		}
	}
#endif

	// TODO: Error reporting.

	return e;
}
#endif
