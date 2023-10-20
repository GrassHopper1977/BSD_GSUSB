#include <time.h>
#include <stdio.h>
#include "utils/timestamp.h"


double diff_timespec(const struct timespec *time1, const struct timespec *time2) {
  return (time1->tv_sec - time2->tv_sec) + ((time1->tv_nsec - time2->tv_nsec) * 1e-9);
}

int main() {
  clock_t start_t, end_t;
  double total_t;
  int i;

  printf("CLOCK_PER_SEC = %d\n", CLOCKS_PER_SEC);

  start_t = clock();
  printf("Start. start_t = %d\n", start_t);

  printf("Big loop. start_t = %d\n", start_t);
  for(i=0; i < 10000000; i++) {
  }
  end_t = clock();
  printf("End of big loop. end_t = %d\n", end_t);

  total_t = (double)(end_t - start_t) / CLOCKS_PER_SEC;
  printf("Total time = %fsecs\n", total_t);

  uint64_t start = nanos();
  start_t = clock();
  printf("Start 2. start = %lu\n", start);

  printf("Big loop 2. start = %lu\n", start);
  for(i=0; i < 10000000; i++) {
  }
  uint64_t end = nanos();
  printf("End of big loop 2. end = %lu\n", end);

  printf("Total time = %luns\n", end-start);
  double total = (double)(end - start) /1000000000L;
  printf("Total time = %fsecs\n", total);

  struct timespec ts_start;
  clock_gettime(CLOCK_MONOTONIC_PRECISE, &ts_start);
  printf("Start 3.      ts_start = %lis %luns\n", ts_start.tv_sec, ts_start.tv_nsec);

  printf("Big loop 3.   ts_start = %lis %luns\n", ts_start.tv_sec, ts_start.tv_nsec);
  for(i=0; i < 10000000; i++) {
  }
  struct timespec ts_end;
  clock_gettime(CLOCK_MONOTONIC_PRECISE, &ts_end);
  printf("End of big loop ts_end = %lis %luns\n", ts_end.tv_sec, ts_end.tv_nsec);

  printf("Total time = %fsecs\n", diff_timespec(&ts_end, &ts_start));
  printf("Total time = %fns\n", diff_timespec(&ts_end, &ts_start) * 1000000000L);
}
