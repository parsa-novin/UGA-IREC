/**
 * @file    hil_config.h
 * @brief   Hardware-In-the-Loop (HIL) mode compile-time switch.
 *
 * Uncomment #define HIL_MODE to build for HIL testing with hil_gui.py.
 *
 * ── What changes in HIL mode ──────────────────────────────────────────────
 *  • UART4 (C5 upstream RS422 link) is left idle — no IRQ, no polling.
 *  • USART2 RX is repurposed to receive 108-byte aggregate packets from the
 *    HIL GUI running on a PC.  Any byte on USART2 that does NOT start with
 *    the 0xA55A frame header is passed to the normal ASCII command handler,
 *    so "H", "67", "UP", "DOWN", "ALTxxxxx", etc. still work from the same
 *    serial terminal.
 *  • USART2 TX no longer sends binary aggregate telemetry frames (which
 *    would corrupt the ASCII feedback the GUI reads).  USART1 still sends
 *    aggregate frames if a separate ground station is connected.
 *  • A 10 Hz status line is emitted on USART2 so hil_gui.py can track the
 *    H5's estimated altitude, velocity, and commanded deployment angle:
 *        [HIL] t=<ms> alt=<m>m vel=<mps>mps deploy=<0-1> angle=<deg>deg
 *
 * ── Wiring ───────────────────────────────────────────────────────────────
 *    H5 PA2 (USART2 TX) → RX of USB-UART adapter → PC
 *    H5 PA3 (USART2 RX) → TX of USB-UART adapter → PC
 *    115 200 baud, 8N1
 *
 * ── Usage ────────────────────────────────────────────────────────────────
 *   1. Uncomment the line below.
 *   2. Rebuild and flash the H5.
 *   3. Select the matching COM port in hil_gui.py (source = "rocketpy" or
 *      "synth") and press Start Injection.
 *   4. To return to flight operation: re-comment and rebuild.
 */

#ifndef HIL_CONFIG_H
#define HIL_CONFIG_H

//#define HIL_MODE  /* enable in h5hil project only */

#endif /* HIL_CONFIG_H */
