#include "devices/timer.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include "threads/interrupt.h"
#include "threads/io.h"
#include "threads/synch.h"
#include "threads/thread.h"

/* See [8254] for hardware details of the 8254 timer chip. */

#if TIMER_FREQ < 19
#error 8254 timer requires TIMER_FREQ >= 19
#endif
#if TIMER_FREQ > 1000
#error TIMER_FREQ <= 1000 recommended
#endif


static struct list sleep_list; 
/* Number of timer ticks since OS booted. */
static int64_t ticks;

/* Number of loops per timer tick.
   Initialized by timer_calibrate(). */
static unsigned loops_per_tick;

static intr_handler_func timer_interrupt;
static bool too_many_loops (unsigned loops);
static void busy_wait (int64_t loops);
static void real_time_sleep (int64_t num, int32_t denom);


/* Sets up the 8254 Programmable Interval Timer (PIT) to
   interrupt PIT_FREQ times per second, and registers the
   corresponding interrupt. */
/* 1초에 TIMER_FREQ(100)번 타이머 인터럽트가 들어오고 그 순간에 하나의 작업을 한다.
헷갈리지 말 것 타이머 인터럽트가 발생하는 그 작은 순간에 알람확인 ,타임슬라이스 확인같은 작은 일만 하는거고
타임슬라이스만료, 우선순위 변경 등의 사유가 없다면 다시 원래 작업을 계속하는 것
*/
void
timer_init (void) {
	/* 8254 input frequency divided by TIMER_FREQ, rounded to
	   nearest. */
	uint16_t count = (1193180 + TIMER_FREQ / 2) / TIMER_FREQ;

	outb (0x43, 0x34);    /* CW: counter 0, LSB then MSB, mode 2, binary. */
	outb (0x40, count & 0xff);
	outb (0x40, count >> 8);

	list_init(&sleep_list);
	/* 타이머가 울리면 timer_interrupt 발생*/
	intr_register_ext (0x20, timer_interrupt, "8254 Timer");

}

/* sleep_list 정렬용: 더 빨리 깰 스레드가 먼저 오도록 */
static bool wake_less (const struct list_elem *a,
                       const struct list_elem *b,
                       void *aux UNUSED) {
  const struct thread *ta = list_entry (a, struct thread, sleep_elem);
  const struct thread *tb = list_entry (b, struct thread, sleep_elem);
  return ta->wake_tick < tb->wake_tick;
}


/* Calibrates loops_per_tick, used to implement brief delays. */
void
timer_calibrate (void) {
	unsigned high_bit, test_bit;

	ASSERT (intr_get_level () == INTR_ON);
	printf ("Calibrating timer...  ");

	/* Approximate loops_per_tick as the largest power-of-two
	   still less than one timer tick. */
	loops_per_tick = 1u << 10;
	while (!too_many_loops (loops_per_tick << 1)) {
		loops_per_tick <<= 1;
		ASSERT (loops_per_tick != 0);
	}

	/* Refine the next 8 bits of loops_per_tick. */
	high_bit = loops_per_tick;
	for (test_bit = high_bit >> 1; test_bit != high_bit >> 10; test_bit >>= 1)
		if (!too_many_loops (high_bit | test_bit))
			loops_per_tick |= test_bit;

	printf ("%s'"PRIu64" loops/s.\n", (uint64_t) loops_per_tick * TIMER_FREQ);
}

/* Returns the number of timer ticks since the OS booted. */
int64_t
timer_ticks (void) {
	enum intr_level old_level = intr_disable ();
	int64_t t = ticks;
	intr_set_level (old_level);
	barrier ();
	return t;
}

/* Returns the number of timer ticks elapsed since THEN, which
   should be a value once returned by timer_ticks(). */
int64_t
timer_elapsed (int64_t then) {
	return timer_ticks () - then;
}

/* Suspends execution for approximately TICKS timer ticks. */
// void
// timer_sleep (int64_t ticks) {
// 	int64_t start = timer_ticks ();

// 	ASSERT (intr_get_level () == INTR_ON);
// 	/* 깨울 시간이 됐나? */
// 	/* 깨울 시간 = ticks, 현재 시간 */
// 	while (timer_elapsed (start) < ticks)
// 		thread_yield ();
// }

/*애초에 인자로*/
void timer_sleep (int64_t ticks) {
  if (ticks <= 0) return;
  ASSERT (intr_get_level () == INTR_ON);

  struct thread *t = thread_current();
  int64_t wake = timer_ticks() + ticks;

  enum intr_level old = intr_disable();                  
  t->wake_tick = wake;
  list_insert_ordered(&sleep_list, &t->sleep_elem, wake_less, NULL);
  thread_block();                                        // BLOCKED
  intr_set_level(old);                                   // 복구
}




/* Suspends execution for approximately MS milliseconds. */
void
timer_msleep (int64_t ms) {
	real_time_sleep (ms, 1000);
}

/* Suspends execution for approximately US microseconds. */
void
timer_usleep (int64_t us) {
	real_time_sleep (us, 1000 * 1000);
}

/* Suspends execution for approximately NS nanoseconds. */
void
timer_nsleep (int64_t ns) {
	real_time_sleep (ns, 1000 * 1000 * 1000);
}

/* Prints timer statistics. */
void
timer_print_stats (void) {
	printf ("Timer: %"PRId64" ticks\n", timer_ticks ());
}

/* Timer interrupt handler. */
/* 타이머가 울리면 타이머가 울린 횟수를 증가시켜준다 
 * ticks++ : 타이머가 한번 더 딸깍했다 !
 */
static void
timer_interrupt (struct intr_frame *args UNUSED) {
  ticks++;
  thread_tick();

  int64_t now = ticks;  // ← 이 값을 쓰자

  while (!list_empty(&sleep_list)) {
    struct thread *t = list_entry(list_front(&sleep_list),
                                  struct thread, sleep_elem);
    if (t->wake_tick <= now) {        // ← timer_ticks() 대신 now
      list_pop_front(&sleep_list);
      thread_unblock(t);
      // (알람-우선순위용 선점은 나중에 추가)
    } else break;
  }
}



/* Returns true if LOOPS iterations waits for more than one timer
   tick, otherwise false. */
static bool
too_many_loops (unsigned loops) {
	/* Wait for a timer tick. */
	int64_t start = ticks;
	while (ticks == start)
		barrier ();

	/* Run LOOPS loops. */
	start = ticks;
	busy_wait (loops);

	/* If the tick count changed, we iterated too long. */
	barrier ();
	return start != ticks;
}

/* Iterates through a simple loop LOOPS times, for implementing
   brief delays.

   Marked NO_INLINE because code alignment can significantly
   affect timings, so that if this function was inlined
   differently in different places the results would be difficult
   to predict. */
static void NO_INLINE
busy_wait (int64_t loops) {
	while (loops-- > 0)
		barrier ();
}

/* Sleep for approximately NUM/DENOM seconds. */
static void
real_time_sleep (int64_t num, int32_t denom) {
	/* Convert NUM/DENOM seconds into timer ticks, rounding down.

	   (NUM / DENOM) s
	   ---------------------- = NUM * TIMER_FREQ / DENOM ticks.
	   1 s / TIMER_FREQ ticks
	   */
	int64_t ticks = num * TIMER_FREQ / denom;

	ASSERT (intr_get_level () == INTR_ON);
	if (ticks > 0) {
		/* We're waiting for at least one full timer tick.  Use
		   timer_sleep() because it will yield the CPU to other
		   processes. */
		timer_sleep (ticks);
	} else {
		/* Otherwise, use a busy-wait loop for more accurate
		   sub-tick timing.  We scale the numerator and denominator
		   down by 1000 to avoid the possibility of overflow. */
		ASSERT (denom % 1000 == 0);
		busy_wait (loops_per_tick * num / 1000 * TIMER_FREQ / (denom / 1000));
	}
}
