#pragma once

#include "LinkedBlockingQueue.h"

typedef struct _QUEUED_DECODE_UNIT {
    DECODE_UNIT decodeUnit;
    LINKED_BLOCKING_QUEUE_ENTRY entry;
} QUEUED_DECODE_UNIT, *PQUEUED_DECODE_UNIT;

#pragma pack(push, 1)

// The encrypted video header must be a multiple
// of 16 bytes in size to ensure the block size
// for FEC stays a multiple of 16 too.
typedef struct _ENC_VIDEO_HEADER {
    uint8_t iv[12];
    uint32_t frameNumber;
    uint8_t tag[16];
} ENC_VIDEO_HEADER, *PENC_VIDEO_HEADER;

#define FLAG_CONTAINS_PIC_DATA 0x1
#define FLAG_EOF 0x2
#define FLAG_SOF 0x4

typedef struct _NV_VIDEO_PACKET {
    uint32_t streamPacketIndex;
    uint32_t frameIndex;
    uint8_t flags;
    uint8_t reserved;
    uint8_t multiFecFlags;
    uint8_t multiFecBlocks;
    uint32_t fecInfo;
} NV_VIDEO_PACKET, *PNV_VIDEO_PACKET;

#define FLAG_EXTENSION 0x10

#define FIXED_RTP_HEADER_SIZE 12
#define MAX_RTP_HEADER_SIZE 16

typedef struct _RTP_PACKET {
    uint8_t header;
    uint8_t packetType;
    uint16_t sequenceNumber;
    uint32_t timestamp;
    uint32_t ssrc;
} RTP_PACKET, *PRTP_PACKET;

// Fields are big-endian
typedef struct _SS_PING {
    char payload[16];
    uint32_t sequenceNumber;
} SS_PING, *PSS_PING;

// Fields are big-endian
#define SS_FRAME_FEC_PTYPE 0x5502
typedef struct _SS_FRAME_FEC_STATUS {
    uint32_t frameIndex;
    uint16_t highestReceivedSequenceNumber;
    uint16_t nextContiguousSequenceNumber;
    uint16_t missingPacketsBeforeHighestReceived;
    uint16_t totalDataPackets;
    uint16_t totalParityPackets;
    uint16_t receivedDataPackets;
    uint16_t receivedParityPackets;
    uint8_t fecPercentage;
    uint8_t multiFecBlockIndex;
    uint8_t multiFecBlockCount;
} SS_FRAME_FEC_STATUS, *PSS_FRAME_FEC_STATUS;

// Frame header discriminator bytes. The first byte of the frame header on the
// SOF packet selects the header layout. 0x01/0x81 are the stock GFE/Sunshine
// values handled in VideoDepacketizer.c.
//
// The Apollo 2.0 latency trace variant is the stock discriminator PLUS ONE:
// 0x01 -> 0x02 and 0x81 -> 0x82. It is arithmetic, not a bit set; setting bit 0
// of either stock value would be a no-op. The parser recovers the stock value
// with a matching subtraction.
//
// A traced header is byte-identical to the stock header of the same form, with
// a SS_FRAME_TIMESTAMP_EXT appended immediately after it. The host MUST NOT
// emit these unless the client advertised ML_FF_LATENCY_TRACE, because a client
// that does not understand them will compute the wrong frame header size and
// corrupt the bitstream.
//
// Both forms are defined and parsed. Apollo only ever emits the short header,
// so FRAME_HDR_DISC_SHORT_TRACE is the only one it needs; the long form is
// accepted so a host that adopts the 0x81 header later does not silently break.
#define FRAME_HDR_DISC_SHORT           0x01
#define FRAME_HDR_DISC_LONG            0x81
#define FRAME_HDR_DISC_SHORT_TRACE     0x02
#define FRAME_HDR_DISC_LONG_TRACE      0x82

// Highest extension version this client understands. Advertised to the host in
// the x-ml-general.traceExtVersion SDP attribute so the host can emit the
// highest version both sides support. See artifacts/wire-contract-frame-trace.md.
//
// The client accepts any version from 1 up to this, because a host that predates
// a bump keeps emitting the older one and that must keep working.
#define SS_FRAME_TIMESTAMP_EXT_VERSION 2

// Bits in SS_FRAME_TIMESTAMP_EXT.validityMask (v2 and later).
//
// A clear bit means the host had no measurement for that stage on this frame --
// NOT that the stage took zero time. Zero is a legal CLOCK_MONOTONIC value, so
// there is no in-band sentinel and the mask is the only way to tell them apart.
// Skip an unflagged field: do not convert it, and do not substitute a nearby
// stamp for it.
//
// The host genuinely cannot stamp everything on every frame. captureRequestedUs
// is absent on the synchronous capture path, which has no request hook, and
// captureCompleteUs is absent on every repeated frame emitted by the minimum-FPS
// static-content path. Requiring all five would empty the host half of the trace
// on every software-encoder session and every static screen.
#define SS_STAMP_VALID_CAPTURE_REQUESTED  0x01
#define SS_STAMP_VALID_CAPTURE_COMPLETE   0x02
#define SS_STAMP_VALID_ENCODE_SUBMIT      0x04
#define SS_STAMP_VALID_ENCODE_COMPLETE    0x08
#define SS_STAMP_VALID_TX_PIPELINE_ENTRY  0x10
#define SS_STAMP_VALID_ALL                0x1F

// In-band host timestamps for the SPEC.md §3 per-frame latency trace. Appended
// to the SOF frame header when the trace capability is negotiated in both
// directions. All fields are little-endian, matching the rest of the frame
// header. All timestamps are host CLOCK_MONOTONIC microseconds; they are joined
// to client timestamps via the clock offset estimated on the control stream.
//
// frameIndex is echoed so the client can reject an extension that has been
// mis-parsed or torn, rather than silently emitting a bogus trace row.
//
// THE SIZE OF THIS STRUCT MUST NOT CHANGE without both peers landing the change
// simultaneously. VideoDepacketizer.c advances frameHeaderSize by sizeof(*this)
// to locate the picture data BEFORE it checks extVersion, so a peer that
// disagrees about the size mis-locates the picture data and corrupts the
// bitstream. Keeping v2 at 48 bytes is what makes a version mismatch cost only
// the host columns instead of the stream: a v1 client skips the correct 48
// bytes, fails the version check, and carries on. Fail safe, not fail corrupt.
typedef struct _SS_FRAME_TIMESTAMP_EXT {
    uint8_t extVersion;
    uint8_t validityMask; // v1: was reserved[0], always sent as 0
    uint8_t reserved[2];
    uint32_t frameIndex;
    uint64_t captureRequestedUs;
    uint64_t captureCompleteUs;
    uint64_t encodeSubmitUs;
    uint64_t encodeCompleteUs;
    // NOT the first packet's transmit time and it cannot be: the frame header is
    // inside the FEC-protected payload, so parity is computed over it before any
    // packet is handed to the socket. Between this stamp and the real first send
    // the host still does Reed-Solomon parity for up to 4 FEC blocks, AES-GCM
    // encryption of every shard when video encryption is on, and an intra-frame
    // pacing sleep. Do NOT compute one-way delay from it. The host CSV carries
    // the true value plus the measured bias distribution.
    uint64_t txPipelineEntryUs;
} SS_FRAME_TIMESTAMP_EXT, *PSS_FRAME_TIMESTAMP_EXT;

// The size rule above is load-bearing enough that it should not rely on anyone
// reading the comment. Fails the build rather than corrupting a bitstream.
typedef char SS_FRAME_TIMESTAMP_EXT_size_check[
    (sizeof(SS_FRAME_TIMESTAMP_EXT) == 48) ? 1 : -1];

// Apollo 2.0 clock synchronisation, SPEC.md §3. NTP-style four-timestamp
// exchange over the existing ENet control channel. Fields are little-endian to
// match the enclosing control packet payload convention.
#define SS_CLOCK_SYNC_REQUEST_PTYPE  0x3010
#define SS_CLOCK_SYNC_RESPONSE_PTYPE 0x3011

typedef struct _SS_CLOCK_SYNC_REQUEST {
    uint32_t sequenceNumber;
    uint32_t reserved;
    uint64_t clientTxUs; // t1, client CLOCK_MONOTONIC
} SS_CLOCK_SYNC_REQUEST, *PSS_CLOCK_SYNC_REQUEST;

typedef struct _SS_CLOCK_SYNC_RESPONSE {
    uint32_t sequenceNumber;
    uint32_t reserved;
    uint64_t clientTxUs;  // t1, echoed verbatim
    uint64_t hostRxUs;    // t2, host CLOCK_MONOTONIC
    uint64_t hostTxUs;    // t3, host CLOCK_MONOTONIC
} SS_CLOCK_SYNC_RESPONSE, *PSS_CLOCK_SYNC_RESPONSE;

#pragma pack(pop)
