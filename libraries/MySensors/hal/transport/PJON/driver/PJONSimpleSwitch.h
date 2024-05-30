
/*-O//\         __     __
  |-gfo\       |__| | |  | |\ | ®
  |!y°o:\      |  __| |__| | \| 13.0
  |y"s§+`\     multi-master, multi-media bus network protocol
 /so+:-..`\    Copyright 2010-2020 by Giovanni Blu Mitolo gioscarab@gmail.com
 |+/:ngr-*.`\
 |5/:%&-a3f.:;\
 \+//u/+g%{osv,,\
   \=+&/osw+olds.\\
      \:/+-.-°-:+oss\
       | |       \oy\\
       > <
______-| |-__________________________________________________________________

PJONSimpleSwitch has been contributed by Fred Larsen.

It routes packets between buses with different bus ids, and also between
local buses with no bus ids. It is limited to one single strategy in contrast
to its descendant PJONSwitch.

A default gateway can be specified, identifying one of the attached buses to
receive packets to other target buses than any of the attached buses. It is
possible to use a PJONSimpleSwitch to handle leaf buses in a tree structure.

NAT (network address translation) support is present, allowing a local bus
to communicate with shared buses. To do this, the local bus must have the
bus id set to a public/NAT bus id with which it can be reached from other
buses, but be set to local mode and not shared mode.
Any incoming packet to the public bus id will be forwarded into the local bus
with the receiver bus id changed to 0.0.0.0 which is considered equivalent
with a local bus address.
Outgoing packets must be sent in shared mode with a sender bus id of 0.0.0.0,
which will be replaced with the NAT address when forwarded by the switch,
enabling receivers on shared buses to reply back to the local bus.
_____________________________________________________________________________

This software is experimental and it is distributed "AS IS" without any
warranty, use it at your own risk.

Copyright 2010-2020 by Giovanni Blu Mitolo gioscarab@gmail.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#pragma once
#include "PJON.h"

#ifndef PJON_ROUTER_MAX_BUSES
#define PJON_ROUTER_MAX_BUSES 5
#endif

template<class Strategy>
class PJONSimpleSwitch
{
protected:
	uint8_t bus_count = 0;
	uint8_t default_gateway = PJON_NOT_ASSIGNED;
	uint8_t current_bus = PJON_NOT_ASSIGNED;
	PJON<Strategy> *buses[PJON_ROUTER_MAX_BUSES];

	void connect(
	    uint8_t bus_count_in,
	    PJON<Strategy> * const buses_in[],
	    uint8_t default_gateway_in,
	    void *custom_pointer,
	    PJON_Receiver receiver,
	    PJON_Error error
	)
	{
		bus_count = (bus_count_in > PJON_ROUTER_MAX_BUSES) ?
		            PJON_ROUTER_MAX_BUSES : bus_count_in;
		default_gateway = default_gateway_in;
		for(uint8_t i = 0; i < bus_count; i++) {
			buses[i] = buses_in[i];
			buses[i]->set_receiver(receiver);
			buses[i]->set_error(error);
			buses[i]->set_custom_pointer(custom_pointer);
			buses[i]->set_router(true);
		}
	};

	uint8_t find_attached_bus_with_id(
	    const uint8_t *bus_id,
	    const uint8_t /*device_id*/,
	    uint8_t &start_bus
	)
	{
		for(uint8_t i=start_bus; i<bus_count; i++) {
			if(memcmp(bus_id, buses[i]->tx.bus_id, 4) == 0) {
				start_bus = i + 1; // Continue searching for more matches after this
				return i; // Explicit bus id match
			}
		}
		start_bus = PJON_NOT_ASSIGNED;
		return PJON_NOT_ASSIGNED;
	};

#ifdef PJON_ROUTER_NEED_INHERITANCE
	virtual
#endif
	void send_packet(const uint8_t *payload, const uint16_t length,
	                 const uint8_t receiver_bus, const uint8_t sender_bus,
	                 bool &ack_sent, const PJON_Packet_Info &packet_info)
	{
		// Send an ACK once to notify that the packet will be delivered
		if(
		    !ack_sent &&
		    (packet_info.header & PJON_ACK_REQ_BIT) &&
		    (packet_info.rx.id != PJON_BROADCAST)
		) {
			buses[sender_bus]->strategy.send_response(PJON_ACK);
			ack_sent = true;
		}

		/* Set current_bus to receiver bus before potentially calling error
		   callback for that bus */
		uint8_t send_bus = current_bus;
		current_bus = receiver_bus;

		/* NAT support: If a shared packet comes from a local bus destined to a
		   non-local receiver, then put the NAT address of the bus as the sender
		   bus id so that replies can find the route back via NAT. */
		PJON_Packet_Info p_info = packet_info;
		if ((packet_info.header & PJON_MODE_BIT) &&
		        !(buses[sender_bus]->config & PJON_MODE_BIT) &&
		        memcmp(buses[sender_bus]->tx.bus_id, PJONTools::localhost(), 4)!=0 &&
		        memcmp(packet_info.tx.bus_id, PJONTools::localhost(), 4)==0) {
			// Replace sender bus id with public/NAT bus id in the packet
			memcpy(&p_info.tx.bus_id, buses[sender_bus]->tx.bus_id, 4);
		}

		/* NAT support: If a shared packet comes with receiver bus id matching the
		   NAT address of a local bus, then change the receiver bus id to 0.0.0.0
		   before forwarding the shared packet to the local bus. */
		if ((packet_info.header & PJON_MODE_BIT) &&
		        !(buses[receiver_bus]->config & PJON_MODE_BIT) &&
		        memcmp(buses[receiver_bus]->tx.bus_id, PJONTools::localhost(), 4)!=0 &&
		        memcmp(packet_info.rx.bus_id, buses[receiver_bus]->tx.bus_id, 4)==0) {
			// Replace receiver bus id with 0.0.0.0 when sending to local bus
			memcpy(p_info.rx.bus_id, PJONTools::localhost(), 4);
		}

		uint16_t result =
#if PJON_MAX_PACKETS == 0
		    buses[receiver_bus]->forward_blocking(
		        p_info,
		        (const uint8_t *)payload,
		        length
		    );
		/* Call error function explicitly, because it is not called by
		   forward_blocking */
		if(result == PJON_FAIL) {
			dynamic_error_function(PJON_CONNECTION_LOST, 0);
		}
#else
		    buses[receiver_bus]->forward(
		        p_info,
		        (const uint8_t *)payload,
		        length
		    );
#endif
		current_bus = send_bus;
	}

	void forward_packet(
	    const uint8_t *payload,
	    const uint16_t length,
	    const uint8_t receiver_bus,
	    const uint8_t sender_bus,
	    bool &ack_sent,
	    const PJON_Packet_Info &packet_info
	)
	{
		// If receiving bus matches and not equal to sending bus, then route packet
		if(receiver_bus != PJON_NOT_ASSIGNED && receiver_bus != sender_bus)
			send_packet(
			    payload,
			    length,
			    receiver_bus,
			    sender_bus,
			    ack_sent,
			    packet_info
			);
	}

#ifdef PJON_ROUTER_NEED_INHERITANCE
	virtual
#endif
	uint8_t find_bus_with_id(
	    const uint8_t bus_id[],
	    const uint8_t device_id,
	    uint8_t &start_bus
	)
	{
		return find_attached_bus_with_id(bus_id, device_id, start_bus);
	};

#ifdef PJON_ROUTER_NEED_INHERITANCE
	virtual
#endif
	void dynamic_receiver_function(
	    uint8_t *payload,
	    uint16_t length,
	    const PJON_Packet_Info &packet_info
	)
	{
		uint8_t start_search = 0;
		bool ack_sent = false; // Send ACK once even if delivering to multiple buses
		do {
			uint8_t receiver_bus = find_bus_with_id((const uint8_t*)
			                                        ((packet_info.header & PJON_MODE_BIT) != 0 ?
			                                                packet_info.rx.bus_id : PJONTools::localhost()),
			                                        packet_info.rx.id, start_search
			                                       );

			/* The NAT case:
			A. A shared packet comes in destined to the "public" bus id registered
			   on the local bus. It will be found normally, and send_packet will
			   modify the receiver bus id to 0.0.0.0 before sending.
			B. A shared packet comes in from the local bus, with sender bus id
			   0.0.0.0 and a valid receiver bus id. The receiver bus id will be
			   found normally, and send_packet will modify the sender bus id
			   from 0.0.0.0 to the registered public/NAT bus id of the local bus.
			*/

			if(receiver_bus == PJON_NOT_ASSIGNED) {
				receiver_bus = default_gateway;
			}

			forward_packet(
			    payload,
			    length,
			    receiver_bus,
			    current_bus,
			    ack_sent,
			    packet_info
			);

		} while(start_search != PJON_NOT_ASSIGNED);
	};

#ifdef PJON_ROUTER_NEED_INHERITANCE
	virtual
#endif
	void dynamic_error_function(uint8_t code, uint16_t data) { }

public:

	PJONSimpleSwitch() {};

	PJONSimpleSwitch(
	    uint8_t bus_count,
	    PJON<Strategy> * const buses[],
	    uint8_t default_gateway = PJON_NOT_ASSIGNED
	)
	{
		connect_buses(bus_count, buses, default_gateway);
	};

	// Specialized constructor to simplify syntax when using 2 buses
	PJONSimpleSwitch(
	    PJON<Strategy> &bus0,
	    PJON<Strategy> &bus1,
	    uint8_t default_gateway = PJON_NOT_ASSIGNED
	)
	{
		PJON<Strategy> *buses[2] = { &bus0, &bus1 };
		connect_buses(2, buses, default_gateway);
	};

	// Specialized constructor to simplify syntax when using 3 buses
	PJONSimpleSwitch(
	    PJON<Strategy> &bus0,
	    PJON<Strategy> &bus1,
	    PJON<Strategy> &bus2,
	    uint8_t default_gateway = PJON_NOT_ASSIGNED
	)
	{
		PJON<Strategy> *buses[3] = { &bus0, &bus1, &bus2 };
		connect_buses(3, buses, default_gateway);
	};

	void begin()
	{
		for(uint8_t i = 0; i < bus_count; i++) {
			buses[i]->begin();
		}
	};

	void loop()
	{
		for(current_bus = 0; current_bus < bus_count; current_bus++) {
			uint16_t code =
			    buses[current_bus]->receive(
			        buses[current_bus]->strategy.get_receive_time()
			    );
			if(PJON_MAX_PACKETS < bus_count && code == PJON_ACK) {
				break;
			}
		}
		for(current_bus = 0; current_bus < bus_count; current_bus++) {
			buses[current_bus]->update();
		}
		current_bus = PJON_NOT_ASSIGNED;
	};

	void connect_buses(
	    uint8_t bus_count_in,
	    PJON<Strategy> * const buses_in[],
	    uint8_t default_gateway_in = PJON_NOT_ASSIGNED
	)
	{
		connect(
		    bus_count_in,
		    buses_in,
		    default_gateway_in,
		    this,
		    PJONSimpleSwitch<Strategy>::receiver_function,
		    PJONSimpleSwitch<Strategy>::error_function
		);
	};

	// Return the position of the bus currently calling a callback.
	// (It may return PJON_NOT_ASSIGNED if not doing a callback.)
	uint8_t get_callback_bus() const
	{
		return current_bus;
	}

	// Return one of the buses, in the same order as sent to the constructor
	PJON<Strategy> &get_bus(const uint8_t ix)
	{
		return *(buses[ix]);
	}

	static void receiver_function(
	    uint8_t *payload,
	    uint16_t length,
	    const PJON_Packet_Info &packet_info
	)
	{
		(
		    (PJONSimpleSwitch<Strategy>*)packet_info.custom_pointer
		)->dynamic_receiver_function(
		    payload,
		    length,
		    packet_info
		);
	}

	static void error_function(uint8_t code, uint16_t data, void *custom_pointer)
	{
		((PJONSimpleSwitch<Strategy>*)custom_pointer)->dynamic_error_function(
		    code,
		    data
		);
	}
};
