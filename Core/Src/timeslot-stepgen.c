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

// Pre-calculated:
const int MarksPerPulses[] = {65535, 40704, 20352, 0 , 1696, 1628 , 0};

int Mark = MarksPerPulses[4]; // actually 24 = 1696
int Tally = 0;

const int PointsPerMinislot = 128;

static int CurrentMinislot = 0;

static const int MinislotsPerTimeframe = 320;
static const int Minislot_DirectionChange = 0;
static const int Minislot_ShortCycle  = MinislotsPerTimeframe - 1;
static const int Minislot_NormalCycle = MinislotsPerTimeframe;
static const int Minislot_LongCycle   = MinislotsPerTimeframe + 1;

void update(void)
{
	if (CurrentMinislot == 0)
	{
		// Adjust direction pins
	}
	else if (CurrentMinislot <= 318)
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
	else if (CurrentMinislot == Minislot_ShortCycle)
	{
	    SignalLow();
		if (ShortCycle)	NextTimeframe();
	}
	else if (CurrentMinislot == Minislot_NormalCycle)
	{
		if (!LongCycle)	NextTimeframe();
	}
	else // Long cycle
	{
		NextTimeframe();
	}
	CurrentMinislot++;
}

