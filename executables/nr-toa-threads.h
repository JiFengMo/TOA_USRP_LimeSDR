#pragma once

/**
 * OAI-style positioning branch thread / actor labels.
 * UE runtime uses two work threads: RX/control and sync/measure/solver.
 */
#define TOA_THREAD_ANCHOR_MAIN "ANCHOR_main_thread"
#define TOA_THREAD_RX_CONTROL "rx_control_thread"
#define TOA_THREAD_SYNC_ACTOR "sync_actor"
