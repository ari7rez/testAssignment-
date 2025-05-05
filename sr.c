/* sr.c */
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT 16.0      /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6  /* the maximum number of buffered unacked packet \
                        MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 7    /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1) /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
static int ComputeChecksum(struct pkt packet)
{
  int checksum = packet.seqnum + packet.acknum;
  for (int i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);
  return checksum;
}

static bool IsCorrupted(struct pkt packet)
{
  return packet.checksum != ComputeChecksum(packet);
}

/********* Sender (A) variables and procedures ************/
static struct pkt buffer[WINDOWSIZE];
static bool acked[WINDOWSIZE];
static int windowfirst, windowlast;
static int windowcount;
static int A_nextseqnum;

void A_init(void)
{
  A_nextseqnum = 0;
  windowfirst = 0;
  windowlast = -1;
  windowcount = 0;
  for (int i = 0; i < WINDOWSIZE; i++)
    acked[i] = false;
}

void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  if (windowcount < WINDOWSIZE)
  {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    windowlast = (windowlast + 1) % WINDOWSIZE;
    buffer[windowlast] = sendpkt;
    acked[windowlast] = false;
    windowcount++;

    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    if (windowcount == 1)
      starttimer(A, RTT);

    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  else
  {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}

void A_input(struct pkt packet)
{
  if (!IsCorrupted(packet))
  {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    // find ACK in buffer
    for (int i = 0; i < windowcount; i++)
    {
      int idx = (windowfirst + i) % WINDOWSIZE;
      if (buffer[idx].seqnum == packet.acknum)
      {
        if (!acked[idx])
        {
          if (TRACE > 0)
            printf("----A: ACK %d is not a duplicate\n", packet.acknum);
          new_ACKs++;
          acked[idx] = true;

          // ✅ Always slide over contiguous ACKs
          stoptimer(A);
          while (windowcount > 0 && acked[windowfirst])
          {
            acked[windowfirst] = false;
            windowfirst = (windowfirst + 1) % WINDOWSIZE;
            windowcount--;
          }
          if (windowcount > 0)
            starttimer(A, RTT);
        }
        else
        {
          if (TRACE > 0)
            printf("----A: duplicate ACK received, do nothing!\n");
        }
        break;
      }
    }
  }
  else if (TRACE > 0)
  {
    printf("----A: corrupted ACK is received, do nothing!\n");
  }
}

void A_timerinterrupt(void)
{
  if (TRACE > 0)
    printf("----A: time out, resend all unACKed packets in buffer\n");

  for (int i = 0; i < windowcount; i++)
  {
    int idx = (windowfirst + i) % WINDOWSIZE;
    if (!acked[idx])
    {
      printf("---A: resending packet %d\n", buffer[idx].seqnum);
      tolayer3(A, buffer[idx]);
      packets_resent++;
    }
  }

  starttimer(A, RTT); // Always restart timer
}

/********* Receiver (B) variables and procedures ************/
static struct pkt rbuffer[WINDOWSIZE];
static bool rcvd[WINDOWSIZE];
static int expectedseqnum;
static int B_nextseqnum;

void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;
  for (int i = 0; i < WINDOWSIZE; i++)
    rcvd[i] = false;
}

void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int seq = packet.seqnum;
  int diff = (seq - expectedseqnum + SEQSPACE) % SEQSPACE;
  bool inWindow = (diff < WINDOWSIZE);
  bool newPkt = false;

  if (IsCorrupted(packet) || !inWindow)
  {
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");

    // Send duplicate ACK for last correctly received packet
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum = (B_nextseqnum + 1) % SEQSPACE;
    sendpkt.acknum = (expectedseqnum - 1 + SEQSPACE) % SEQSPACE;
    for (int i = 0; i < 20; i++)
      sendpkt.payload[i] = '0';
    sendpkt.checksum = ComputeChecksum(sendpkt);
    tolayer3(B, sendpkt);
    return; // Drop corrupted or invalid packet silently — do NOT ACK
  }

  if (!rcvd[seq % WINDOWSIZE])
  {
    if (TRACE > 0)
    {
      if (diff == 0)
        printf("----B: packet %d is correctly received, send ACK!\n", seq);
      else
        printf("----B: packet %d correctly received but out of order, buffered!\n", seq);
    }
    packets_received++;
    rbuffer[seq % WINDOWSIZE] = packet;
    rcvd[seq % WINDOWSIZE] = true;
    newPkt = true;
  }
  else
  {
    if (TRACE > 0)
      printf("----B: duplicate packet %d, already buffered, resend ACK!\n", seq);
  }

  /* send ACK for whatever seq we got */
  sendpkt.seqnum = B_nextseqnum;
  B_nextseqnum = (B_nextseqnum + 1) % SEQSPACE;
  sendpkt.acknum = seq;
  for (int i = 0; i < 20; i++)
    sendpkt.payload[i] = '0';
  sendpkt.checksum = ComputeChecksum(sendpkt);
  tolayer3(B, sendpkt);

  /* deliver in-order packets only when we received a new one */
  if (newPkt)
  {
    while (rcvd[expectedseqnum % WINDOWSIZE])
    {
      tolayer5(B, rbuffer[expectedseqnum % WINDOWSIZE].payload);
      rcvd[expectedseqnum % WINDOWSIZE] = false;
      expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
    }
  }
}
/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/
void B_output(struct msg message) {}
void B_timerinterrupt(void) {}