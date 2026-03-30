#pragma once

/**
 * OAI-style positioning branch thread / actor labels.
 * Main + clock monitor + TOA UE + sync/track/measure actors + solver + trace.
 */
#define TOA_THREAD_ANCHOR_MAIN "ANCHOR_main_thread"
#define TOA_THREAD_CLOCK_MON "clock_mon_thread"
#define TOA_THREAD_TOA_UE "TOA_UE_thread"
#define TOA_THREAD_SYNC_ACTOR "sync_actor"
#define TOA_THREAD_TRACK_ACTOR "track_actor"
#define TOA_THREAD_MEASURE_ACTOR "measure_actor"
#define TOA_THREAD_SOLVER "solver_thread"
#define TOA_THREAD_STAT "stat_thread"
#define TOA_THREAD_TRACE "trace_thread"
