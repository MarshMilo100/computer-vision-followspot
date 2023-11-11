#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "e131.c"

#define __cplusplus

class Test
{
    int main()
    {
        int data [512];
        int pan1 = 50;
        int tilt1 = 50;
        int pan2 = 100;
        int tilt2 = 100;

        int socket;
        e131_packet_t packet;
        e131_addr_t dest;

        // initialize the new E1.31 packet in universe 1 with 24 slots in preview mode
        e131_pkt_init(&packet, 1, 24);
        memcpy(&packet, "sACN Test Send", 15);
        e131_set_option(&packet, E131_OPT_PREVIEW, true);

        // set remote system destination as unicast address
        e131_unicast_dest(&dest, "192.168.0.105", E131_DEFAULT_PORT);

        uint8_t level = pan1;

        for (;;)
        {
            for (size_t i = 1; i <= 24; i++)
            {
                packet.dmp.prop_val[i] = level;
            }
            
            level = ++level % pan2;
            e131_send(socket, &packet, &dest);
            e131_pkt_dump(stderr, &packet);
            packet.frame.seq_number++;
            Sleep(250);
        }
    }
};