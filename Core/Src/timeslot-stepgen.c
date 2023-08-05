// time slots stepgen
// This kind of stepgen works on the basis of time slots. Each time slot receives an exact given number of pulses, evenly dispersed inside that timeslot.
// This kind of stepgen never undershoots or overshoots. unlike a velocity-based stepgen which overshoots horribly

// Each timeslot is subdivided to N minislots
// The first minislot is dedicated to direction setup. No steps occuring
// The last minislot is dedicated to the falling edge of the step pulse. (Unconfirmed) There needs to be a time gap between a falling edge and a direction change

// Updates (request for next, future timeslot) are expected to occur at the halfway point of a timeslot.
// It is expected that the arrival time of an update will slowly drift. In such case, The entire timeslot will be
// ever so slightly stretched or compressed, so that the arrival time drifts back into the center
// ! AS of now, (for testing) we stretch or compress by adding an extra minislot, or skipping the last minislot. (Not by altering the interval of the timer)
// ! It is not as good, because the pulse area won't stretch as much as it could, making the pulses distribution even more uneven

// N selection guidelines:
// Lower bound: a minislot has to be at least the length of a pulse (+%x for jitter)
// Lower bound 2: a minislot has be large enough to not overwhelm the CPU
// N should be divisible by as many factors as possible. Multiples of 60 is a good start

// Timeslot length guidelines:
// In Mach3 and R&R card, the timeslot length is 5ms.
// It's not good to have a small number of steps-per-timeslot, that corresponds to a high speed,
// because this will introduce unwanted accelerations/vibrations as we move thru the timeslots. Difference between 1 & 2 is much more noticeable than a difference between 3 & 4

// To mitigate, We can add more microsteps so that number gets bigger, or make the time slot bigger
// As of now, We'll try a 1ms to see what happens.

// Uneven division:
// Lets say we have 320 minislots. Lets say we want to fit 37 pulses inside the timeslot.
// If we send a pulse every exactly N minislots, the last pulse will not align well with the end of the timeslot.
// To mitigate, we add a multiplier. Each minislot is now worth 128 points.
// We have a running number called Tally. When the tally reaches or exceeds the "Mark", we send a pulse & decrement

#include <stdint.h>
#include <stdbool.h>
#if 1
#include <stdio.h> // ! for testing
#endif

// Pre-calculated:
#define MAX_PULSES_PER_TIMESLOT 30
static int MarksPerPulses[MAX_PULSES_PER_TIMESLOT] = {0};
static const int PointsPerMinislot = 128;

static const int MinislotsPerTimeframe = 320;
static const int Minislot_DirectionChange = 0;
static const int Minislot_PulseArea   = MinislotsPerTimeframe - 2;
static const int Minislot_ShortCycle  = MinislotsPerTimeframe - 1;
static const int Minislot_NormalCycle = MinislotsPerTimeframe;
static const int Minislot_LongCycle   = MinislotsPerTimeframe + 1;

// Data about the current timeslot
static int CurrentMinislot = 0;
static int Mark = 0;
static int Tally = 0;
static bool ShortCycle = false;
static bool LongCycle = false;

static int StepsForNextTimeframe = 0;

#if 1
int pulsesForThisTimeframe = 0;
#endif

void SignalHigh(void);
void SignalLow(void);
void NextTimeframe(void);

void update(void)
{
	if (CurrentMinislot == 0)
	{
		// Adjust direction pins
	}
	else if (CurrentMinislot < Minislot_PulseArea)
	{
		Tally += PointsPerMinislot;
		if (Tally >= Mark)
		{
			Tally -= Mark;
			SignalHigh();
		}
		else
		{
			SignalLow();
		}
	}
	else if (CurrentMinislot < Minislot_ShortCycle)
	{
	    SignalLow();
		if (ShortCycle)	NextTimeframe();
	}
	else if (CurrentMinislot < Minislot_NormalCycle)
	{
		if (!LongCycle)	NextTimeframe();
	}
	else // Long cycle
	{
		NextTimeframe();
	}
	CurrentMinislot++;
}

void NextTimeframe(void)
{
	CurrentMinislot = -1;
	Tally = 0;
	Mark = MarksPerPulses[StepsForNextTimeframe]; // ! TEST actually 24 = 1696
	#if 1
	pulsesForThisTimeframe = StepsForNextTimeframe;
	printf("NextTimeframe()\n");
	#endif
}

#if 0
GPIO_TypeDef* table_gpios_pulses[4]  = {XP_GPIO_Port, YP_ORIG_GPIO_Port, ZP_GPIO_Port, AP_ORIG_GPIO_Port};
uint32_t   table_pins_pulses_set[4]  = {XP_Pin,       YP_ORIG_Pin,       ZP_Pin,       AP_ORIG_Pin};
uint32_t table_pins_pulses_reset[4]  = {XP_Pin << 16, YP_ORIG_Pin << 16, ZP_Pin << 16, AP_ORIG_Pin << 16};

GPIO_TypeDef* table_gpios_dir[4]  = {XD_GPIO_Port, YD_GPIO_Port, ZD_GPIO_Port, AD_GPIO_Port};
uint32_t   table_pins_dir_set[4]  = {XD_Pin,       YD_Pin,       ZD_Pin,       AD_Pin};
uint32_t table_pins_dir_reset[4]  = {XD_Pin << 16, YD_Pin << 16, ZD_Pin << 16, AD_Pin << 16};


static inline void step_hi(int axis)
{
    table_gpios_pulses[axis]->BSRR = table_pins_pulses_set[axis];
}

static inline void step_lo(int axis)
{
    table_gpios_pulses[axis]->BSRR = table_pins_pulses_reset[axis];
}

static inline void dir_hi(int axis)
{
    table_gpios_dir[axis]->BSRR = table_pins_dir_set[axis];
}

static inline void dir_lo(int axis)
{
    table_gpios_dir[axis]->BSRR = table_pins_dir_reset[axis];
}
#else
#include <stdio.h> // ! for testing
static int pinstatus = 0;
void SignalHigh(void)
{
	if (!pinstatus) printf("SignalHigh() at %d (%d) (Next: %d)\n", CurrentMinislot, pulsesForThisTimeframe, StepsForNextTimeframe);
	pinstatus = 1;
}
void SignalLow(void)
{
	if (pinstatus) printf("SignalLow() at %d (%d) (Next: %d)\n", CurrentMinislot, pulsesForThisTimeframe, StepsForNextTimeframe);
	pinstatus = 0;
}
#endif

#if 1 // TEST code

void ComputeMarksPerPulses(void)
{
	MarksPerPulses[0] = 65535;
	const int windowlen = Minislot_PulseArea-1;
	for (int i = 1; i < MAX_PULSES_PER_TIMESLOT; i++)
	{
		MarksPerPulses[i] = (PointsPerMinislot * windowlen) / i;
	}
}

int main()
{
	ComputeMarksPerPulses();
	//
	StepsForNextTimeframe = 0;
	NextTimeframe(); CurrentMinislot++;
	//
	Mark = MarksPerPulses[StepsForNextTimeframe];
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < Minislot_LongCycle; j++)
		{
			update();
		}
		StepsForNextTimeframe = i;
	}
}

#endif

