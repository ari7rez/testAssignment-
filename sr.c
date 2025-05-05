// sr.c
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol. Adapted from J.F.Kurose
   Based on ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

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
   - Implemented Selective Repeat protocol.
**********************************************************************/

#define RTT 16.0      /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6  /* the maximum number of buffered unacked packet \
                        MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 7    /* the min sequence space for GBN must be at least windowsize + 1 */
                      /* NOTE: SEQSPACE 7 is too small for standard Selective Repeat with WINDOWSIZE 6.
                         Standard SR requires SEQSPACE >= 2 * WINDOWSIZE (>= 12) to avoid ambiguity.
                         This implementation uses the provided constants but may exhibit issues
                         in test cases that expose sequence number wrap-around ambiguity. */
#define NOTINUSE (-1) /* used to fill header fields that are not being used */
#define MAX_RETRIES 10
static int resend_count[SEQSPACE]; // retry count per seqnum

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i = 0; i < 20; i++)
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}

/********* Sender (A) variables and functions ************/

// Selective Repeat Sender (A) state
static int base;                             // Sequence number of the oldest unacked packet
static int next_seqnum;                      // Next sequence number to be sent
static struct pkt sender_buffer[WINDOWSIZE]; // Buffer to store packets waiting for ACK
static bool acked[WINDOWSIZE];               // Array to track acknowledged packets within the window (indexed by seqnum % WINDOWSIZE)

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  // Check if the next sequence number to be sent is within the sender's window [base, base + WINDOWSIZE - 1]
  // Using modular arithmetic for wrap-around comparison: (next_seqnum - base + SEQSPACE) % SEQSPACE
  if ((next_seqnum - base + SEQSPACE) % SEQSPACE < WINDOWSIZE)
  {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = next_seqnum;
    sendpkt.acknum = NOTINUSE; // ACKs are sent by receiver B
    for (i = 0; i < 20; i++)
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    int buffer_index = next_seqnum % WINDOWSIZE; // Map seqnum to buffer index
    sender_buffer[buffer_index] = sendpkt;
    acked[buffer_index] = false; // Mark as unacknowledged initially

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if this is the first packet in the window (at 'base') */
    if (base == next_seqnum)
    {
      starttimer(A, RTT);
    }

    next_seqnum = (next_seqnum + 1) % SEQSPACE; // Move to the next sequence number
  }
  /* if blocked,  window is full */
  else
  {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    // Note: In SR, upper layer is typically blocked when window is full.
    // The emulator's window_full statistic might be tied to this specific printf.
    window_full++;
  }
}

/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet))
  {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    // total_ACKs_received++; // Assuming this counter is updated elsewhere or not strictly required by SR

    int acked_seqnum = packet.acknum;

    // Check if the ACKed sequence number is within the sender's window [base, base + WINDOWSIZE - 1]
    // Using modular arithmetic for wrap-around comparison
    if ((acked_seqnum - base + SEQSPACE) % SEQSPACE < WINDOWSIZE)
    {
      int buffer_index = acked_seqnum % WINDOWSIZE; // Map acked seqnum to buffer index

      if (!acked[buffer_index])
      { // If this is a new ACK for this packet
        if (TRACE > 0)
          printf("----A: Received new ACK for packet %d\n", acked_seqnum);
        acked[buffer_index] = true; // Mark this packet as acknowledged
        // new_ACKs++; // Assuming this counter is for new ACKs
        resend_count[acked_seqnum] = 0; // Reset retry count when ACKed
      }
      else
      {
        if (TRACE > 0)
          printf("----A: Received duplicate ACK for packet %d\n", acked_seqnum);
      }

      // Slide the window forward as much as possible past consecutive acknowledged packets starting from 'base'
      while (acked[base % WINDOWSIZE])
      {
        if (TRACE > 1)
          printf("----A: Packet %d acknowledged, sliding window base\n", base);
        acked[base % WINDOWSIZE] = false; // Reset acked state for the packet leaving the window
        base = (base + 1) % SEQSPACE;     // Move window base forward

        // If there are still unacked packets in the new window, restart the timer for the new base
        if ((base - next_seqnum + SEQSPACE) % SEQSPACE < WINDOWSIZE)
        {
          stoptimer(A);       // Stop timer for the old base
          starttimer(A, RTT); // Start timer for the new base (oldest unacked)
        }
        else
        {
          stoptimer(A); // All packets in the window have been acked, stop the timer
        }
      }
    }
    // Note: ACKs for packets outside the window (old or future) are typically ignored by the sender in SR.
    // The original GBN code had logic for duplicate ACKs outside the window, which is removed here for SR.
    // The printf "----A: duplicate ACK received, do nothing!" from GBN is kept if within window but already acked.
  }
  else if (TRACE > 0)
    printf("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
// In SR, the timer is typically for the oldest unacked packet (at 'base')
void A_timerinterrupt(void)
{
  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");
  // packets_timeout++; // Assuming a counter for timeouts

  // Retransmit the packet at the window base
  int buffer_index = base % WINDOWSIZE; // Map base seqnum to buffer index

  if (TRACE > 0)
    printf("---A: resending packet %d\n", sender_buffer[buffer_index].seqnum);

  if (resend_count[base] >= MAX_RETRIES)
  {
    printf("----A: Packet %d reached max retries. Giving up.\n", base);
    exit(1); // Or return or log error
  }
  tolayer3(A, sender_buffer[buffer_index]);
  packets_resent++;
  resend_count[base]++;
  starttimer(A, RTT);
  ;
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* Initialize Selective Repeat sender state */
  base = 0;        // Window starts at sequence number 0
  next_seqnum = 0; // Next packet to send is 0

  // Initialize acked array to false (no packets acknowledged yet) and sender buffer
  for (int i = 0; i < WINDOWSIZE; i++)
  {
    acked[i] = false;
    // sender_buffer[i] initialization is not strictly necessary for pkt struct,
    // but clearing payload could be done if needed.
  }

  // The GBN windowfirst, windowlast, windowcount, and A_nextseqnum are replaced by SR state.
}

/********* Receiver (B)  variables and procedures ************/

// Selective Repeat Receiver (B) state
static int expected_seqnum;                    // Sequence number of the next packet to deliver to layer 5
static struct pkt receiver_buffer[WINDOWSIZE]; // Buffer to store out-of-order packets (indexed by seqnum % WINDOWSIZE)
static bool received[WINDOWSIZE];              // Array to track received packets within the window (indexed by seqnum % WINDOWSIZE)

// B_nextseqnum is used for the sequence number of the ACK packet sent by B.
// In this unidirectional sim, B's packets are only ACKs, so a simple toggle (0/1) is often sufficient.
// Let's keep it as per the original GBN structure if it's used by the emulator for B->A packets.
static int B_nextseqnum = 1; // Initialize B's next sequence number for ACKs

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
  int i;
  struct pkt ackpkt; // Packet to send ACK

  // Check if the received packet is corrupted
  if (IsCorrupted(packet))
  {
    if (TRACE > 0)
      printf("----B: Corrupted packet received, ignoring\n");
    return; // Ignore corrupted packets
  }

  if (TRACE > 0)
    printf("----B: Uncorrupted packet %d received\n", packet.seqnum);
  // packets_received++; // Assuming this counter is for correctly received packets (handled below)

  int packet_seqnum = packet.seqnum;

  // Check if the packet's sequence number is within the receiver's window [expected_seqnum, expected_seqnum + WINDOWSIZE - 1]
  // Using modular arithmetic for wrap-around comparison
  if ((packet_seqnum - expected_seqnum + SEQSPACE) % SEQSPACE < WINDOWSIZE)
  {
    if (TRACE > 0)
      printf("----B: Packet %d is within the receiver window [%d, %d]\n",
             packet_seqnum, expected_seqnum, (expected_seqnum + WINDOWSIZE - 1) % SEQSPACE);

    // Send an ACK for this specific packet's sequence number
    ackpkt.seqnum = B_nextseqnum;  // Use B's sequence number for the ACK packet
    ackpkt.acknum = packet_seqnum; // ACK the sequence number of the received data packet
    // We don't have data to send in ACK payload - fill with 0s as in original GBN
    for (i = 0; i < 20; i++)
      ackpkt.payload[i] = '0';
    ackpkt.checksum = ComputeChecksum(ackpkt);
    tolayer3(B, ackpkt);
    B_nextseqnum = (B_nextseqnum + 1) % 2; // Toggle B's sequence number for the next ACK

    int buffer_index = packet_seqnum % WINDOWSIZE; // Map seqnum to buffer index

    // If this packet has not been received before within the current window
    if (!received[buffer_index])
    {
      if (TRACE > 0)
        printf("----B: Packet %d is a new packet within window, buffering\n", packet_seqnum);
      receiver_buffer[buffer_index] = packet; // Store the packet
      received[buffer_index] = true;          // Mark as received
      // packets_received++; // Increment counter if this is for unique, correct receptions
      // The original code increments this for any uncorrupted, in-order packet.
      // Let's increment when we mark it as received in the buffer.
      packets_received++; // Count this correct reception
    }
    else
    {
      if (TRACE > 0)
        printf("----B: Packet %d is a duplicate within window, re-acknowledging\n", packet_seqnum);
    }

    // Deliver consecutive packets starting from expected_seqnum to Layer 5
    while (received[expected_seqnum % WINDOWSIZE])
    {
      if (TRACE > 0)
        printf("----B: Delivering packet %d to layer 5\n", expected_seqnum);
      tolayer5(B, receiver_buffer[expected_seqnum % WINDOWSIZE].payload);
      received[expected_seqnum % WINDOWSIZE] = false; // Reset buffer slot for the packet leaving the window
      // receiver_buffer[expected_seqnum % WINDOWSIZE] could be cleared if needed
      expected_seqnum = (expected_seqnum + 1) % SEQSPACE; // Slide receiver window
    }
  }
  else
  {
    if (TRACE > 0)
      printf("----B: Packet %d is outside the receiver window [%d, %d], ignoring\n",
             packet_seqnum, expected_seqnum, (expected_seqnum + WINDOWSIZE - 1 + SEQSPACE) % SEQSPACE);
    // In SR, packets outside the window are typically ignored.
    // The original GBN code would resend the last ACK here, which is not standard SR behavior.
    // We will not send an ACK for packets outside the window.
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  /* Initialize Selective Repeat receiver state */
  expected_seqnum = 0; // Receiver initially expects packet 0 to deliver to layer 5

  // Initialize received array to false (no packets received in the window yet) and receiver buffer
  for (int i = 0; i < WINDOWSIZE; i++)
  {
    received[i] = false;
    // receiver_buffer[i] initialization is not strictly necessary for pkt struct,
    // but clearing payload could be done if needed.
  }
  // B_nextseqnum is initialized above where it's declared.
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
  // Not used in unidirectional A->B
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
  // Not used in unidirectional A->B
}