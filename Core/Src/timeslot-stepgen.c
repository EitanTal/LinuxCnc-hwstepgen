#if 0
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

// TODO:
// Initial Tally divided by Mark, is effectively phase. Is there an advantage to starting (and ending) the phase at 50%?

#include <stdint.h>
#include <stdbool.h>
#if 1
#include <stdio.h> // ! for testing
#endif


#define AXES 4


#define MAX_PULSES_PER_TIMESLOT 159

// Pre-calculated:
static int MarksPerPulses[MAX_PULSES_PER_TIMESLOT+1] = {  // Calculated for window size of (317 * 128)
	65535, 40576, 20288, 13525, 10144, 8115, 6762, 5796, 5072, 4508, 4057, 3688, 3381, 3121, 2898, 2705, 2536, 2386, 2254, 2135,
	2028, 1932, 1844, 1764, 1690, 1623, 1560, 1502, 1449, 1399, 1352, 1308, 1268, 1229, 1193, 1159, 1127, 1096, 1067, 1040,
	1014, 989, 966, 943, 922, 901, 882, 863, 845, 828, 811, 795, 780, 765, 751, 737, 724, 711, 699, 687, 676, 665, 654, 644,
	634, 624, 614, 605, 596, 588, 579, 571, 563, 555, 548, 541, 533, 526, 520, 513, 507, 500, 494, 488, 483, 477, 471, 466,
	461, 455, 450, 445, 441, 436, 431, 427, 422, 418, 414, 409, 405, 401, 397, 393, 390, 386, 382, 379, 375, 372, 368, 365,
	362, 359, 355, 352, 349, 346, 343, 340, 338, 335, 332, 329, 327, 324, 322, 319, 317, 314, 312, 309, 307, 305, 302, 300,
	298, 296, 294, 291, 289, 287, 285, 283, 281, 279, 277, 276, 274, 272, 270, 268, 266, 265, 263, 261, 260, 258, 256, 0};
static const int PointsPerMinislot = 128;

static const int MinislotsPerTimeframe = 320;
static const int Minislot_DirectionChange = 0;
static const int Minislot_PulseArea   = MinislotsPerTimeframe - 2;
static const int Minislot_ShortCycle  = MinislotsPerTimeframe - 1;
static const int Minislot_NormalCycle = MinislotsPerTimeframe;
static const int Minislot_LongCycle   = MinislotsPerTimeframe + 1;

// Data about the current timeslot
static int CurrentMinislot = 0;
static int Mark[AXES] = {0};
static int Direction[AXES] = {0};
static int Tally[AXES] = {0};
static int TotalPulses[AXES] = {0};
static bool ShortCycle = false;
static bool LongCycle = false;

static int StepsForNextTimeframe = 0;

#if 1
int pulsesForThisTimeframe = 0;
int pulsesInTimeframe = 0;
#endif

static inline void SignalHigh(int axis);
static inline void SignalLow(int axis);
static inline void DirectionHigh(int axis);
static inline void DirectionLow(int axis);

static inline void NextTimeframe(void);

void update(void)
{
	if (CurrentMinislot == 0) // Adjust direction pins
	{
		for (int i = 0; i < AXES; i++)
		{
			if (Direction[i] < 0)
			{
				DirectionHigh(i);
			}
			else
			{
				DirectionLow(i);
			}
		}
	}
	else if (CurrentMinislot < Minislot_PulseArea)
	{
		for (int i = 0; i < AXES; i++)
		{
			Tally[i] += PointsPerMinislot;
			if (Tally[i] >= Mark[i])
			{
				Tally[i] -= Mark[i];
				SignalHigh(i);
				TotalPulses[i] += Direction[i];
			}
			else
			{
				SignalLow(i);
			}
		}
	}
	else if (CurrentMinislot < Minislot_ShortCycle)
	{
		for (int i = 0; i < AXES; i++)
		{
	    	SignalLow(i);
		}
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
	for (int i = 0; i < AXES; i++)
	{
		Tally[i] = 0;
		Mark[i] = MarksPerPulses[StepsForNextTimeframe];
	}
	#if 1
	pulsesForThisTimeframe = StepsForNextTimeframe;
	printf("NextTimeframe() . Pulses sent in this one: %d\n", pulsesInTimeframe);
	pulsesInTimeframe = 0;
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

void SignalHigh(int axis)
{
	if (!pinstatus)
	{
		//printf("SignalHigh() at %d (%d) (Next: %d)\n", CurrentMinislot, pulsesForThisTimeframe, StepsForNextTimeframe);
	}
	pinstatus = 1;
}
void SignalLow(int axis)
{
	if (pinstatus) 
	{
		//printf("SignalLow() at %d (%d) (Next: %d)\n", CurrentMinislot, pulsesForThisTimeframe, StepsForNextTimeframe);
		pulsesInTimeframe++;
	}
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
	for (int i = 0; i < MAX_PULSES_PER_TIMESLOT; i++)
	{
		for (int j = 0; j < Minislot_NormalCycle; j++)
		{
			if (CurrentMinislot == Minislot_NormalCycle/2)
			{
				StepsForNextTimeframe = i;
			}
			update();
		}
	}
}

#endif

#endif
