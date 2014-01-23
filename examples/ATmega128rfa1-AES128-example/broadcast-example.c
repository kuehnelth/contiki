/*
 * Copyright (c) 2011, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "sys/etimer.h"
#include "net/uip.h"
#include "net/uip-ds6.h"

#include "simple-udp.h"


#include <stdio.h>
#include <string.h>

#include "dev/leds.h"

#define UDP_PORT 1234

#define SEND_INTERVAL		(5 * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

static struct simple_udp_connection broadcast_connection;

/*---------------------------------------------------------------------------*/
PROCESS(broadcast_example_process, "UDP broadcast example process");
AUTOSTART_PROCESSES(&broadcast_example_process);
/*---------------------------------------------------------------------------*/
static void
receiver(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  /*printf("Data received from: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x (from port %d) on port %d  with length %d\n",
         sender_addr->u16[0],sender_addr->u16[1],sender_addr->u16[2],sender_addr->u16[3],sender_addr->u16[4],sender_addr->u16[5],sender_addr->u16[6],sender_addr->u16[7], 
	 sender_port, receiver_port, datalen);*/
 printf("Broadcast received from: %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x on port %d\n", 
	sender_addr->u8[0],sender_addr->u8[1],sender_addr->u8[2],sender_addr->u8[3],sender_addr->u8[4],sender_addr->u8[5],sender_addr->u8[6],sender_addr->u8[7], sender_addr->u8[8],sender_addr->u8[9],sender_addr->u8[10],sender_addr->u8[11],sender_addr->u8[12],sender_addr->u8[13],sender_addr->u8[14],sender_addr->u8[15], sender_port);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_example_process, ev, data)
{
  static struct etimer periodic_timer;
  static struct etimer send_timer;
  uip_ipaddr_t addr;
  //uip_ip6addr(&addr, 0xfe01, 0, 0, 0, 0, 0, 0, 2);
  //uip_ip6addr(&addr, 0xfe80, 0, 0, 0, 0, 0, 0, 2);

  PROCESS_BEGIN();

#if PLATFORM_HAS_LEDS
  leds_off(LEDS_ALL);
#endif

  simple_udp_register(&broadcast_connection, UDP_PORT,
                      NULL, UDP_PORT,
                      receiver);

  etimer_set(&periodic_timer, SEND_INTERVAL);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));
    etimer_reset(&periodic_timer);
    etimer_set(&send_timer, SEND_TIME);

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&send_timer));
    uip_create_linklocal_allnodes_mcast(&addr);
#if PLATFORM_HAS_LEDS
  leds_on(LEDS_ALL);
#endif
    /*printf("Sending broadcast from: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x on port %d\n", 
	addr.u16[0],addr.u16[1],addr.u16[2],addr.u16[3],addr.u16[4],addr.u16[5],addr.u16[6],addr.u16[7], 
	UDP_PORT);*/
    printf("Sending broadcast from: %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x on port %d\n", addr.u8[0],addr.u8[1],addr.u8[2],addr.u8[3],addr.u8[4],addr.u8[5],addr.u8[6],addr.u8[7], addr.u8[8],addr.u8[9],addr.u8[10],addr.u8[11],addr.u8[12],addr.u8[13],addr.u8[14],addr.u8[15], UDP_PORT);
    //uip_create_linklocal_allnodes_mcast(&addr);
    simple_udp_sendto(&broadcast_connection, "Test", 4, &addr);
#if PLATFORM_HAS_LEDS
  leds_off(LEDS_ALL);
#endif
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
