#include "Limelight-internal.h"

// Uncomment to test 3 byte Annex B start sequences with GFE
//#define FORCE_3_BYTE_START_SEQUENCES

static PLENTRY nalChainHead;
static PLENTRY nalChainTail;
static int nalChainDataLength;

static unsigned int nextFrameNumber;
static unsigned int startFrameNumber;
static bool waitingForNextSuccessfulFrame;
static bool waitingForIdrFrame;
static bool waitingForRefInvalFrame;
static unsigned int lastPacketInStream;
static bool decodingFrame;
static int frameType;
static uint16_t lastPacketPayloadLength;
static bool strictIdrFrameWait;
static uint64_t syntheticPtsBase;
static uint16_t frameHostProcessingLatency;
static uint64_t firstPacketReceiveTime;
static unsigned int firstPacketPresentationTime;
static bool dropStatePending;
static bool idrFrameProcessed;

// Latency trace state for the frame currently being reassembled (SPEC.md §3).
// Owned exclusively by the video receive thread: set on the SOF packet, read and
// cleared in reassembleFrame(). No other thread touches these.
static bool frameTraceValid;
static SS_FRAME_TIMESTAMP_EXT frameTraceExt;
static uint64_t frameTraceLastPacketRxUs;
static bool frameTraceLastPacketRxValid;

// Version actually observed on the wire, plus once-per-session log latches so a
// mismatch is reported without spamming a per-frame path.
static uint8_t frameTraceNegotiatedVersion;
static bool frameTraceVersionLogged;
static bool frameTraceVersionMismatchLogged;
// --- Adaptive jitter buffer (SPEC.md §4 Item C) ---------------------------
//
// There is no jitter measurement anywhere in this tree today. RtpVideoQueue has
// no time-based hold and no depth bound, and the renderer's two "adaptive" drop
// thresholds read an EWMA that is initialised once to a fraction of the vsync
// period and never assigned again. So this starts by actually measuring.
//
// What is measured: per-FRAME inter-arrival, not per-packet. RtpVideoQueue
// deliberately stamps every packet in a frame with the first packet's receive
// time, so per-packet arrival information does not survive to here.
//
// The client is on Wi-Fi and the host is wired, so essentially all observed
// jitter originates on the client's wireless leg or the AP. It is modelled as a
// single-sided noise source: only late arrivals matter, because an early frame
// costs nothing. Spread is measured upward from the window median rather than as
// an absolute deviation, which would let a burst of early frames inflate it.
//
// The baseline is the MEDIAN OF THE MEASURED WINDOW, not the requested frame
// rate. Measuring against StreamConfig.fps would report a host that delivers 60
// while 120 was requested as ~8.3 ms of "jitter" on every single frame, forever,
// on a perfectly clean link. That is a frame-rate mismatch, not jitter, and no
// amount of buffering fixes it. Against the median it correctly reads as zero.
//
// Sizing is a high percentile minus that median, not a mean or an EWMA:
// wireless inter-arrival is heavy-tailed, and a mean of a heavy-tailed
// distribution under-buffers exactly when buffering matters.
//
// The target is the sum of two components, so that the percentile actually
// governs rather than being overridden:
//
//   percentile component - p95 minus median, recomputed every 16 frames, decays
//                          by a bounded step. This is the steady-state level.
//   fast bump            - a bounded transient raised by a single late frame or
//                          a detected loss, halved every recompute.
//
// An earlier version let a single frame set the whole target. That is
// running-maximum-with-a-slow-leak, not a percentile: one 25 ms sample pinned
// the target for 6.6 s at the 500 us/recompute leak, during which ~800 frames
// could re-pin it, so on any link with a spike every few seconds the statistic
// never governed at all. Capping the transient and decaying it separately keeps
// the onset response without letting one sample own the control law.
//
// Response is still asymmetric per SPEC.md §4 Item C -- both components rise
// faster than they fall -- because on a wireless link oscillation is worse than
// a slightly oversized tolerance.
//
// Thread ownership: every variable below is touched only by the video receive
// thread, inside processRtpPayload() and reassembleFrame(). Nothing else reads
// or writes them. The value crosses to Java on the decode unit, which is already
// a per-frame handoff, so no shared-state read is needed for display either.
#define JITTER_WINDOW_SIZE 128
#define JITTER_RECOMPUTE_INTERVAL 16
#define JITTER_PERCENTILE_NUM 95
#define JITTER_PERCENTILE_DEN 100

// How far the target may fall per recompute. At 120 fps a recompute happens
// roughly every 133 ms, so this decays about 3.75 ms per second: fast enough to
// recover after a transient, slow enough not to chase noise back down.
#define JITTER_RAMP_DOWN_STEP_US 500

// A detected frame loss forces the target up by this much immediately, on the
// theory that loss on a wireless link is a leading indicator of a jitter burst
// that the percentile window has not caught up with yet.
#define JITTER_LOSS_BUMP_US 2000

// An inter-arrival gap beyond this multiple of the current baseline is a stall,
// a stream restart or a pause, not jitter. Such samples are DISCARDED, not
// clamped: clamping them to the ceiling made them arithmetically identical to an
// unbounded outlier, so seven of them inside one window would pin the percentile
// at the ceiling and then take seconds to decay.
#define JITTER_STALL_MULTIPLE 4

// The stall test is self-referential -- its threshold comes from the baseline,
// and the baseline is the median of samples the test admitted. A sustained
// degradation past the multiple (120 fps collapsing to 24) would otherwise
// discard every sample forever: the window would never update, the baseline
// would never move, and the estimator would freeze, blind and silent, on exactly
// the degradation it exists to notice.
//
// So after this many consecutive discards the gap is reinterpreted as the new
// normal rather than a stall: the window is cleared and the baseline re-anchored
// to the current interval. That is the escape hatch that breaks the loop.
#define JITTER_STALL_ESCAPE_COUNT 16

// Nothing above this is ever admitted as an inter-arrival sample, whatever the
// baseline says. Bounds the re-anchor path so a multi-second pause cannot become
// the new baseline.
#define JITTER_MAX_SANE_INTERVAL_US 1000000

// The transient bump a single late frame or a loss may contribute. Bounding it
// is what keeps the percentile in charge: an unbounded per-frame maximum turns
// the control law into running-max-with-leak, and the window, the sort and the
// p95 index all become decorative.
#define JITTER_FAST_BUMP_MAX_US 8000

// Hard ceiling regardless of configuration.
#define JITTER_TARGET_CEILING_US 50000

static uint32_t jitterDeltaWindowUs[JITTER_WINDOW_SIZE];
static uint32_t jitterScratchUs[JITTER_WINDOW_SIZE];
static int jitterWindowCount;
static int jitterWindowPos;
static int jitterFramesSinceRecompute;
static int jitterConsecutiveStalls;
static uint64_t jitterLastFrameArrivalUs;
static uint32_t jitterNominalIntervalUs; // startup seed only, until the window fills
static uint32_t jitterBaselineUs;        // measured median inter-arrival
static uint32_t jitterMeasuredUs;        // p95 spread above the baseline
static uint32_t jitterPercentileTargetUs; // slow, percentile-governed component
static uint32_t jitterFastBumpUs;        // bounded, fast-decaying transient
static uint32_t jitterTargetUs;          // sum of the two, clamped to the ceiling
static uint32_t jitterTargetCeilingUs;
static uint32_t jitterLossEvents;
static uint32_t jitterStallsDiscarded;
static uint32_t jitterReanchors;
static bool jitterAdaptiveEnabled;

#define DR_CLEANUP -1000

#define CONSECUTIVE_DROP_LIMIT 120
static unsigned int consecutiveFrameDrops;

static LINKED_BLOCKING_QUEUE decodeUnitQueue;

typedef struct _BUFFER_DESC {
    char* data;
    unsigned int offset;
    unsigned int length;
} BUFFER_DESC, *PBUFFER_DESC;

typedef struct _LENTRY_INTERNAL {
    LENTRY entry;
    void* allocPtr;
} LENTRY_INTERNAL, *PLENTRY_INTERNAL;

#define H264_NAL_TYPE(x) ((x) & 0x1F)
#define HEVC_NAL_TYPE(x) (((x) & 0x7E) >> 1)

#define H264_NAL_TYPE_SEI 6
#define H264_NAL_TYPE_SPS 7
#define H264_NAL_TYPE_PPS 8
#define H264_NAL_TYPE_AUD 9
#define H264_NAL_TYPE_FILLER 12
#define HEVC_NAL_TYPE_VPS 32
#define HEVC_NAL_TYPE_SPS 33
#define HEVC_NAL_TYPE_PPS 34
#define HEVC_NAL_TYPE_AUD 35
#define HEVC_NAL_TYPE_FILLER 38
#define HEVC_NAL_TYPE_SEI 39

// Init
void initializeVideoDepacketizer(int pktSize) {
    LbqInitializeLinkedBlockingQueue(&decodeUnitQueue, 15);

    nextFrameNumber = 1;
    startFrameNumber = 0;
    waitingForNextSuccessfulFrame = false;
    waitingForIdrFrame = true;
    waitingForRefInvalFrame = false;
    lastPacketInStream = UINT32_MAX;
    decodingFrame = false;
    syntheticPtsBase = 0;
    frameHostProcessingLatency = 0;
    firstPacketReceiveTime = 0;
    firstPacketPresentationTime = 0;
    lastPacketPayloadLength = 0;
    dropStatePending = false;
    idrFrameProcessed = false;
    strictIdrFrameWait = !isReferenceFrameInvalidationEnabled();

    // Trace extension version state. These are file-scope statics and
    // moonlight-common-c supports reconnecting in the same process, so a stale
    // version from a previous host must not leak into this session's metadata.
    frameTraceNegotiatedVersion = 0;
    frameTraceVersionLogged = false;
    frameTraceVersionMismatchLogged = false;

    // Adaptive jitter buffer. Disabled unless the caller opts in, so the default
    // configuration is byte-identical to stock: no measurement, no added delay.
    memset(jitterDeltaWindowUs, 0, sizeof(jitterDeltaWindowUs));
    jitterWindowCount = 0;
    jitterWindowPos = 0;
    jitterFramesSinceRecompute = 0;
    jitterConsecutiveStalls = 0;
    jitterLastFrameArrivalUs = 0;
    jitterBaselineUs = 0;
    jitterMeasuredUs = 0;
    jitterPercentileTargetUs = 0;
    jitterFastBumpUs = 0;
    jitterTargetUs = 0;
    jitterLossEvents = 0;
    jitterStallsDiscarded = 0;
    jitterReanchors = 0;
    jitterAdaptiveEnabled = StreamConfig.adaptiveLateFrameToleranceMaxMs > 0;
    jitterTargetCeilingUs = jitterAdaptiveEnabled
            ? (uint32_t)StreamConfig.adaptiveLateFrameToleranceMaxMs * 1000u : 0;
    if (jitterTargetCeilingUs > JITTER_TARGET_CEILING_US) {
        jitterTargetCeilingUs = JITTER_TARGET_CEILING_US;
    }
    // Startup seed for the stall test only. Once the window has samples the
    // baseline comes from their median, never from the requested rate.
    jitterNominalIntervalUs = (StreamConfig.fps >= 10 && StreamConfig.fps <= 1000)
            ? (uint32_t)(1000000 / StreamConfig.fps) : 16667;
    if (jitterAdaptiveEnabled) {
        Limelog("Adaptive late-frame tolerance enabled: ceiling %u us, startup interval %u us\n",
                jitterTargetCeilingUs, jitterNominalIntervalUs);
    }
}

uint8_t getFrameTraceExtVersion(void) {
    return frameTraceNegotiatedVersion;
}

// Recombines the two components into the value the renderer consumes.
// Video receive thread only.
static void updateJitterTarget(void) {
    uint32_t combined = jitterPercentileTargetUs + jitterFastBumpUs;

    if (combined > jitterTargetCeilingUs) {
        combined = jitterTargetCeilingUs;
    }
    jitterTargetUs = combined;
}

// Raises the bounded transient. Video receive thread only.
static void addJitterFastBump(uint32_t bumpUs) {
    if (bumpUs > JITTER_FAST_BUMP_MAX_US) {
        bumpUs = JITTER_FAST_BUMP_MAX_US;
    }
    if (bumpUs > jitterFastBumpUs) {
        jitterFastBumpUs = bumpUs;
    }
    updateJitterTarget();
}

// Recomputes the percentile and moves the target toward it. Video receive
// thread only. Called once every JITTER_RECOMPUTE_INTERVAL frames rather than
// per frame: sorting 128 entries at 7.5 Hz is free, doing it at 120 Hz is not.
static void recomputeJitterTarget(void) {
    int count = jitterWindowCount;
    int i, j, medianIdx, pctIdx;
    uint32_t desired;

    if (count < 4) {
        // Too few samples for a median to mean anything.
        return;
    }

    memcpy(jitterScratchUs, jitterDeltaWindowUs, sizeof(uint32_t) * (size_t)count);

    // Insertion sort. The window is small and nearly sorted in practice, so
    // this beats anything with a call overhead per comparison.
    for (i = 1; i < count; i++) {
        uint32_t key = jitterScratchUs[i];
        for (j = i - 1; j >= 0 && jitterScratchUs[j] > key; j--) {
            jitterScratchUs[j + 1] = jitterScratchUs[j];
        }
        jitterScratchUs[j + 1] = key;
    }

    // Baseline is the median inter-arrival actually observed, so a steady rate
    // below the requested one contributes nothing to measured jitter.
    medianIdx = count / 2;
    jitterBaselineUs = jitterScratchUs[medianIdx];

    pctIdx = (count * JITTER_PERCENTILE_NUM) / JITTER_PERCENTILE_DEN;
    if (pctIdx >= count) {
        pctIdx = count - 1;
    }

    // Single-sided spread: how much later than typical the slow tail runs.
    jitterMeasuredUs = (jitterScratchUs[pctIdx] > jitterBaselineUs)
            ? (jitterScratchUs[pctIdx] - jitterBaselineUs) : 0;

    desired = jitterMeasuredUs;
    if (desired > jitterTargetCeilingUs) {
        desired = jitterTargetCeilingUs;
    }

    if (desired > jitterPercentileTargetUs) {
        // Ramp up immediately.
        jitterPercentileTargetUs = desired;
    }
    else if (jitterPercentileTargetUs > desired) {
        // Ramp down slowly.
        uint32_t delta = jitterPercentileTargetUs - desired;
        jitterPercentileTargetUs -= (delta > JITTER_RAMP_DOWN_STEP_US)
                ? JITTER_RAMP_DOWN_STEP_US : delta;
    }

    // The transient decays far faster than the percentile component, so it can
    // cover the onset of a burst without ever becoming the steady-state level.
    jitterFastBumpUs /= 2;

    updateJitterTarget();
}

// Feeds one completed frame's arrival into the estimator.
// Video receive thread only.
static void updateJitterEstimate(uint64_t arrivalUs) {
    uint64_t deltaUs;
    uint32_t baselineUs;
    uint32_t latenessUs;

    if (!jitterAdaptiveEnabled) {
        return;
    }

    if (jitterLastFrameArrivalUs == 0 || arrivalUs <= jitterLastFrameArrivalUs) {
        // First frame of the session, or a non-advancing clock. Neither yields
        // a usable interval.
        jitterLastFrameArrivalUs = arrivalUs;
        return;
    }

    deltaUs = arrivalUs - jitterLastFrameArrivalUs;
    jitterLastFrameArrivalUs = arrivalUs;

    // Until the window has enough samples for a median, fall back to the
    // nominal interval purely as a startup seed for the stall test.
    baselineUs = (jitterBaselineUs != 0) ? jitterBaselineUs : jitterNominalIntervalUs;

    // Discard stalls outright rather than clamping them. A clamped sample is
    // still the largest value in the window and still drags the percentile up;
    // an earlier version clamped to the ceiling, which made a stall exactly as
    // damaging as an unbounded outlier.
    //
    // But the test cannot be trusted indefinitely, because its threshold comes
    // from a baseline built only out of samples it admitted. If the real rate
    // drops past the multiple and stays there, every sample looks like a stall
    // forever and the estimator freezes. So a run of consecutive discards is
    // taken as evidence that the baseline itself is wrong, and the window is
    // re-anchored to the observed interval.
    if (deltaUs > (uint64_t)baselineUs * JITTER_STALL_MULTIPLE) {
        jitterStallsDiscarded++;

        if (++jitterConsecutiveStalls < JITTER_STALL_ESCAPE_COUNT) {
            return;
        }

        // Sustained: treat it as the new normal, not a stall.
        if (deltaUs > JITTER_MAX_SANE_INTERVAL_US) {
            // A pause or a suspend rather than a rate change. Do not let it
            // become the baseline; wait for something plausible.
            jitterConsecutiveStalls = 0;
            return;
        }

        jitterConsecutiveStalls = 0;
        jitterWindowCount = 0;
        jitterWindowPos = 0;
        jitterBaselineUs = (uint32_t)deltaUs;
        jitterReanchors++;
        Limelog("Jitter estimator re-anchored to %u us after %d sustained long intervals\n",
                jitterBaselineUs, JITTER_STALL_ESCAPE_COUNT);
        baselineUs = jitterBaselineUs;
        // Fall through and admit this sample as the first of the new window.
    }
    else {
        jitterConsecutiveStalls = 0;
    }

    jitterDeltaWindowUs[jitterWindowPos] = (uint32_t)deltaUs;
    jitterWindowPos = (jitterWindowPos + 1) % JITTER_WINDOW_SIZE;
    if (jitterWindowCount < JITTER_WINDOW_SIZE) {
        jitterWindowCount++;
    }

    // Per-frame onset response. Waiting for the percentile alone means waiting
    // for ~7 late frames plus up to a recompute interval, 100-200 ms of
    // unprotected stream at the start of a burst, and pure Wi-Fi jitter arrives
    // with no loss at all so the loss bump is not a fast path for it.
    //
    // This feeds the BOUNDED transient, not the target directly. Letting one
    // sample set the target outright made the percentile decorative; capped and
    // fast-decaying, it covers the onset and then gets out of the way.
    latenessUs = (deltaUs > baselineUs) ? (uint32_t)(deltaUs - baselineUs) : 0;
    if (latenessUs > 0) {
        addJitterFastBump(latenessUs);
    }

    if (++jitterFramesSinceRecompute >= JITTER_RECOMPUTE_INTERVAL) {
        jitterFramesSinceRecompute = 0;
        recomputeJitterTarget();
    }
}

// Fast ramp-up on observed loss. Video receive thread only.
static void notifyJitterLoss(void) {
    if (!jitterAdaptiveEnabled) {
        return;
    }

    jitterLossEvents++;
    addJitterFastBump(jitterFastBumpUs + JITTER_LOSS_BUMP_US);
}

// Reports frame loss to the host and feeds the jitter estimator's fast path.
//
// These are wrapped together so they cannot drift apart. The estimator
// originally hooked one of the six paths that detect loss, so the mechanism the
// design called its fast path was absent from the majority of the cases that
// trigger it -- including the corrupt-frame path, which on a lossy wireless link
// is at least as common as a whole-frame gap.
static void reportFrameLoss(uint32_t startFrame, uint32_t endFrame) {
    notifyJitterLoss();
    connectionDetectedFrameLoss(startFrame, endFrame);
}

// Free the NAL chain
static void cleanupFrameState(void) {
    PLENTRY_INTERNAL lastEntry;

    while (nalChainHead != NULL) {
        lastEntry = (PLENTRY_INTERNAL)nalChainHead;
        nalChainHead = lastEntry->entry.next;
        free(lastEntry->allocPtr);
    }

    nalChainTail = NULL;

    nalChainDataLength = 0;
}

// Cleanup frame state and set that we're waiting for an IDR Frame
static void dropFrameState(void) {
    // This may only be called at frame boundaries
    LC_ASSERT(!decodingFrame);

    // We're dropping frame state now
    dropStatePending = false;

    if (strictIdrFrameWait || !idrFrameProcessed || waitingForIdrFrame) {
        // We'll need an IDR frame now if we're in non-RFI mode, if we've never
        // received an IDR frame, or if we explicitly need an IDR frame.
        waitingForIdrFrame = true;
    }
    else {
        waitingForRefInvalFrame = true;
    }

    // Count the number of consecutive frames dropped
    consecutiveFrameDrops++;

    // If we reach our limit, immediately request an IDR frame and reset
    if (consecutiveFrameDrops == CONSECUTIVE_DROP_LIMIT) {
        Limelog("Reached consecutive drop limit\n");

        // Restart the count
        consecutiveFrameDrops = 0;

        // Request an IDR frame
        waitingForIdrFrame = true;
        LiRequestIdrFrame();
    }

    cleanupFrameState();
}

// Cleanup the list of decode units
static void freeDecodeUnitList(PLINKED_BLOCKING_QUEUE_ENTRY entry) {
    PLINKED_BLOCKING_QUEUE_ENTRY nextEntry;

    while (entry != NULL) {
        nextEntry = entry->flink;

        // Complete this with a failure status
        LiCompleteVideoFrame(entry->data, DR_CLEANUP);

        entry = nextEntry;
    }
}

void stopVideoDepacketizer(void) {
    LbqSignalQueueShutdown(&decodeUnitQueue);
}

// Cleanup video depacketizer and free malloced memory
void destroyVideoDepacketizer(void) {
    // Summarise the estimator once, at teardown, so its counters have a real
    // consumer rather than being incremented and never read.
    //
    // In particular a non-zero reanchor count, or a stall count that is a large
    // fraction of the session, is the signature of the baseline having gone
    // stale -- the failure mode the escape hatch in updateJitterEstimate()
    // exists to break. Without this line that could happen for a whole session
    // with no diagnostic anywhere. It is one log line at teardown, never during
    // the stream, so it cannot perturb what it reports.
    if (jitterAdaptiveEnabled) {
        Limelog("Late-frame tolerance summary: baseline %u us, p95 spread %u us, "
                "final tolerance %u us (percentile %u + transient %u), "
                "loss bumps %u, long intervals discarded %u, re-anchors %u\n",
                jitterBaselineUs, jitterMeasuredUs, jitterTargetUs,
                jitterPercentileTargetUs, jitterFastBumpUs,
                jitterLossEvents, jitterStallsDiscarded, jitterReanchors);
    }

    freeDecodeUnitList(LbqDestroyLinkedBlockingQueue(&decodeUnitQueue));
    cleanupFrameState();
}

// NB: This function also ensures an additional byte for the NALU type exists after the start sequence
static bool getAnnexBStartSequence(PBUFFER_DESC current, PBUFFER_DESC startSeq) {
    // We must not get called for other codecs
    LC_ASSERT(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265));

    if (current->length <= 3) {
        return false;
    }

    if (current->data[current->offset] == 0 &&
        current->data[current->offset + 1] == 0) {
        if (current->data[current->offset + 2] == 0) {
            if (current->length > 4 && current->data[current->offset + 3] == 1) {
                // Frame start
                if (startSeq != NULL) {
                    startSeq->data = current->data;
                    startSeq->offset = current->offset;
                    startSeq->length = 4;
                }
                return true;
            }
        }
        else if (current->data[current->offset + 2] == 1) {
            // NAL start
            if (startSeq != NULL) {
                startSeq->data = current->data;
                startSeq->offset = current->offset;
                startSeq->length = 3;
            }
            return true;
        }
    }

    return false;
}

void validateDecodeUnitForPlayback(PDECODE_UNIT decodeUnit) {
    // Frames must always have at least one buffer
    LC_ASSERT(decodeUnit->bufferList != NULL);
    LC_ASSERT(decodeUnit->fullLength != 0);

    // Validate the buffers in the frame
    if (decodeUnit->frameType == FRAME_TYPE_IDR) {
        // IDR frames always start with codec configuration data
        if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
            // H.264 IDR frames should have an SPS, PPS, then picture data
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_SPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->bufferType == BUFFER_TYPE_PPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next != NULL);
        }
        else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
            // HEVC IDR frames should have an VPS, SPS, PPS, then picture data
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_VPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->bufferType == BUFFER_TYPE_SPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next->bufferType == BUFFER_TYPE_PPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next->next != NULL);
        }
        else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_AV1) {
            // We don't parse the AV1 bitstream
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_PICDATA);
        }
        else {
            LC_ASSERT(false);
        }
    }
    else {
        LC_ASSERT(decodeUnit->frameType == FRAME_TYPE_PFRAME);

        // P frames always start with picture data
        LC_ASSERT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_PICDATA);

        // We must not dequeue a P frame before an IDR frame has been successfully processed
        LC_ASSERT(idrFrameProcessed);
    }
}

bool LiWaitForNextVideoFrame(VIDEO_FRAME_HANDLE* frameHandle, PDECODE_UNIT* decodeUnit) {
    PQUEUED_DECODE_UNIT qdu;

    int err = LbqWaitForQueueElement(&decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    validateDecodeUnitForPlayback(&qdu->decodeUnit);

    *frameHandle = qdu;
    *decodeUnit = &qdu->decodeUnit;
    return true;
}

bool LiPollNextVideoFrame(VIDEO_FRAME_HANDLE* frameHandle, PDECODE_UNIT* decodeUnit) {
    PQUEUED_DECODE_UNIT qdu;

    int err = LbqPollQueueElement(&decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    validateDecodeUnitForPlayback(&qdu->decodeUnit);

    *frameHandle = qdu;
    *decodeUnit = &qdu->decodeUnit;
    return true;
}

bool LiPeekNextVideoFrame(PDECODE_UNIT* decodeUnit) {
    PQUEUED_DECODE_UNIT qdu;

    int err = LbqPeekQueueElement(&decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    validateDecodeUnitForPlayback(&qdu->decodeUnit);

    *decodeUnit = &qdu->decodeUnit;
    return true;
}

void LiWakeWaitForVideoFrame(void) {
    LbqSignalQueueUserWake(&decodeUnitQueue);
}

// Cleanup a decode unit by freeing the buffer chain and the holder
void LiCompleteVideoFrame(VIDEO_FRAME_HANDLE handle, int drStatus) {
    PQUEUED_DECODE_UNIT qdu = handle;
    PLENTRY_INTERNAL lastEntry;

    if (drStatus == DR_NEED_IDR) {
        Limelog("Requesting IDR frame on behalf of DR\n");
        requestDecoderRefresh();
    }
    else if (drStatus == DR_OK && qdu->decodeUnit.frameType == FRAME_TYPE_IDR) {
        // Remember that the IDR frame was processed. We can now use
        // reference frame invalidation.
        idrFrameProcessed = true;
    }

    while (qdu->decodeUnit.bufferList != NULL) {
        lastEntry = (PLENTRY_INTERNAL)qdu->decodeUnit.bufferList;
        qdu->decodeUnit.bufferList = lastEntry->entry.next;
        free(lastEntry->allocPtr);
    }

    // We will have stack-allocated entries iff we have a direct-submit decoder
    if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
        free(qdu);
    }
}

static bool isSeqReferenceFrameStart(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == 5;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        switch (HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length])) {
            case 16:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
                return true;

            default:
                return false;
        }
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

static bool isAccessUnitDelimiter(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_AUD;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_AUD;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

static bool isSeiNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_SEI;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_SEI;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

#ifdef LC_DEBUG
static bool isFillerDataNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_FILLER;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_FILLER;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}
#endif

static bool isPictureParameterSetNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_PPS;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_PPS;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

// Advance the buffer descriptor to the start of the next NAL or end of buffer
static void skipToNextNalOrEnd(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    // If we're starting on a NAL boundary, skip to the next one
    if (getAnnexBStartSequence(buffer, &startSeq)) {
        buffer->offset += startSeq.length;
        buffer->length -= startSeq.length;
    }

    // Loop until we find an Annex B start sequence (3 or 4 byte)
    while (!getAnnexBStartSequence(buffer, NULL)) {
        if (buffer->length == 0) {
            // Reached the end of the buffer
            return;
        }

        buffer->offset++;
        buffer->length--;
    }
}

// Advance the buffer descriptor to the start of the next NAL
static void skipToNextNal(PBUFFER_DESC buffer) {
    skipToNextNalOrEnd(buffer);

    // If we skipped all the data, something has gone horribly wrong
    LC_ASSERT(buffer->length > 0);
}

static bool isIdrFrameStart(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_SPS;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_VPS;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

// Reassemble the frame with the given frame number
static void reassembleFrame(int frameNumber) {
    if (nalChainHead != NULL) {
        QUEUED_DECODE_UNIT qduDS;
        PQUEUED_DECODE_UNIT qdu;

        // Use a stack allocation if we won't be queuing this
        if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
            qdu = (PQUEUED_DECODE_UNIT)malloc(sizeof(*qdu));
        }
        else {
            qdu = &qduDS;
        }

        if (qdu != NULL) {
            qdu->decodeUnit.bufferList = nalChainHead;
            qdu->decodeUnit.fullLength = nalChainDataLength;
            qdu->decodeUnit.frameType = frameType;
            qdu->decodeUnit.frameNumber = frameNumber;
            qdu->decodeUnit.frameHostProcessingLatency = frameHostProcessingLatency;
            qdu->decodeUnit.receiveTimeMs = firstPacketReceiveTime;
            qdu->decodeUnit.presentationTimeMs = firstPacketPresentationTime;
            qdu->decodeUnit.enqueueTimeMs = LiGetMillis();

            // Adaptive late-frame tolerance. The frame is fully assembled here,
            // so this is the arrival instant the estimator wants.
            //
            // The tolerance is carried on the decode unit rather than enforced
            // here. Sleeping on this thread would stall FEC processing for
            // packets already in flight, which on a lossy wireless link is
            // precisely the wrong trade. The renderer applies it by widening its
            // own stale-frame threshold; nothing is delayed anywhere.
            updateJitterEstimate(PltGetMicros());
            qdu->decodeUnit.lateFrameToleranceUs = jitterTargetUs;

            // These might be wrong for a few frames during a transition between SDR and HDR,
            // but the effects shouldn't very noticable since that's an infrequent operation.
            //
            // If we start sending this state in the frame header, we can make it 100% accurate.
            qdu->decodeUnit.hdrActive = LiGetCurrentHostDisplayHdrMode();
            qdu->decodeUnit.colorspace = (uint8_t)(qdu->decodeUnit.hdrActive ? COLORSPACE_REC_2020 : StreamConfig.colorSpace);

            // Latency trace (SPEC.md §3). Host timestamps are converted into the
            // client's monotonic epoch here, on the receive thread, so the client
            // never has to reason about two clocks. If no offset estimate is
            // available the whole host half is dropped rather than guessed; the
            // client-only t_last_packet_rx still goes out so the client-side
            // stages remain measurable during clock sync warmup.
            qdu->decodeUnit.traceValid = false;
            qdu->decodeUnit.traceLastPacketRxValid = false;
            qdu->decodeUnit.traceLastPacketRxUs = 0;
            qdu->decodeUnit.traceHostCaptureRequestedUs = 0;
            qdu->decodeUnit.traceHostCaptureCompleteUs = 0;
            qdu->decodeUnit.traceHostEncodeSubmitUs = 0;
            qdu->decodeUnit.traceHostEncodeCompleteUs = 0;
            qdu->decodeUnit.traceHostTxPipelineEntryUs = 0;
            qdu->decodeUnit.traceHostStampMask = 0;

            if (LatencyTraceEnabled) {
                qdu->decodeUnit.traceLastPacketRxUs = frameTraceLastPacketRxUs;
                qdu->decodeUnit.traceLastPacketRxValid = frameTraceLastPacketRxValid;

                if (frameTraceValid) {
                    uint64_t converted;
                    uint8_t mask = frameTraceExt.validityMask;
                    uint8_t outMask = 0;

                    // Per-field, not all-or-nothing. The host cannot stamp every
                    // stage on every frame -- the synchronous capture path has no
                    // request hook, and a repeated static-content frame has no
                    // capture completion -- so requiring all five would empty the
                    // host half of the trace on exactly those sessions.
                    //
                    // A field whose bit is clear is left alone: not converted, not
                    // defaulted to zero, and never substituted from a neighbouring
                    // stamp. It is emitted blank in the CSV.
                    if ((mask & SS_STAMP_VALID_CAPTURE_REQUESTED) &&
                            convertHostToClientMicros(frameTraceExt.captureRequestedUs, &converted)) {
                        qdu->decodeUnit.traceHostCaptureRequestedUs = converted;
                        outMask |= SS_STAMP_VALID_CAPTURE_REQUESTED;
                    }
                    if ((mask & SS_STAMP_VALID_CAPTURE_COMPLETE) &&
                            convertHostToClientMicros(frameTraceExt.captureCompleteUs, &converted)) {
                        qdu->decodeUnit.traceHostCaptureCompleteUs = converted;
                        outMask |= SS_STAMP_VALID_CAPTURE_COMPLETE;
                    }
                    if ((mask & SS_STAMP_VALID_ENCODE_SUBMIT) &&
                            convertHostToClientMicros(frameTraceExt.encodeSubmitUs, &converted)) {
                        qdu->decodeUnit.traceHostEncodeSubmitUs = converted;
                        outMask |= SS_STAMP_VALID_ENCODE_SUBMIT;
                    }
                    if ((mask & SS_STAMP_VALID_ENCODE_COMPLETE) &&
                            convertHostToClientMicros(frameTraceExt.encodeCompleteUs, &converted)) {
                        qdu->decodeUnit.traceHostEncodeCompleteUs = converted;
                        outMask |= SS_STAMP_VALID_ENCODE_COMPLETE;
                    }
                    if ((mask & SS_STAMP_VALID_TX_PIPELINE_ENTRY) &&
                            convertHostToClientMicros(frameTraceExt.txPipelineEntryUs, &converted)) {
                        qdu->decodeUnit.traceHostTxPipelineEntryUs = converted;
                        outMask |= SS_STAMP_VALID_TX_PIPELINE_ENTRY;
                    }

                    // traceValid means "the extension itself was sound and the
                    // clock offset was available", not "every stage is present".
                    // Per-stage presence is traceHostStampMask.
                    qdu->decodeUnit.traceHostStampMask = outMask;
                    qdu->decodeUnit.traceValid = true;
                }
            }

            // Invoke the key frame callback if needed
            if (nalChainHead->bufferType != BUFFER_TYPE_PICDATA || qdu->decodeUnit.frameType == FRAME_TYPE_IDR) {
                qdu->decodeUnit.frameType = FRAME_TYPE_IDR;
                notifyKeyFrameReceived();
            }
            else {
                qdu->decodeUnit.frameType = FRAME_TYPE_PFRAME;
            }
            
            nalChainHead = nalChainTail = NULL;
            nalChainDataLength = 0;

            if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
                if (LbqOfferQueueItem(&decodeUnitQueue, qdu, &qdu->entry) == LBQ_BOUND_EXCEEDED) {
                    Limelog("Video decode unit queue overflow\n");

                    // RFI recovery is not supported here
                    waitingForIdrFrame = true;

                    // Clear NAL state for the frame that we failed to enqueue
                    nalChainHead = qdu->decodeUnit.bufferList;
                    nalChainDataLength = qdu->decodeUnit.fullLength;
                    dropFrameState();

                    // Free the DU we were going to queue
                    free(qdu);

                    // Free all frames in the decode unit queue
                    freeDecodeUnitList(LbqFlushQueueItems(&decodeUnitQueue));

                    // Request an IDR frame to recover
                    LiRequestIdrFrame();
                    return;
                }
            }
            else {
                // Submit the frame to the decoder
                validateDecodeUnitForPlayback(&qdu->decodeUnit);
                LiCompleteVideoFrame(qdu, VideoCallbacks.submitDecodeUnit(&qdu->decodeUnit));
            }

            // Notify the control connection
            connectionReceivedCompleteFrame(frameNumber);

            // Clear frame drops
            consecutiveFrameDrops = 0;

            // Move the start of our (potential) RFI window to the next frame
            startFrameNumber = nextFrameNumber;
        }
    }
}

static int getBufferFlags(char* data, int length) {
    BUFFER_DESC buffer;
    BUFFER_DESC candidate;

    // We only parse H.264 and HEVC bitstreams
    if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265))) {
        return BUFFER_TYPE_PICDATA;
    }

    buffer.data = data;
    buffer.length = (unsigned int)length;
    buffer.offset = 0;

    if (!getAnnexBStartSequence(&buffer, &candidate)) {
        return BUFFER_TYPE_PICDATA;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        switch (H264_NAL_TYPE(candidate.data[candidate.offset + candidate.length])) {
        case H264_NAL_TYPE_SPS:
            return BUFFER_TYPE_SPS;

        case H264_NAL_TYPE_PPS:
            return BUFFER_TYPE_PPS;

        default:
            return BUFFER_TYPE_PICDATA;
        }
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        switch (HEVC_NAL_TYPE(candidate.data[candidate.offset + candidate.length])) {
            case HEVC_NAL_TYPE_SPS:
                return BUFFER_TYPE_SPS;

            case HEVC_NAL_TYPE_PPS:
                return BUFFER_TYPE_PPS;

            case HEVC_NAL_TYPE_VPS:
                return BUFFER_TYPE_VPS;

            default:
                return BUFFER_TYPE_PICDATA;
        }
    }
    else {
        LC_ASSERT(false);
        return BUFFER_TYPE_PICDATA;
    }
}

// As an optimization, we can cast the existing packet buffer to a PLENTRY and avoid
// a malloc() and a memcpy() of the packet data.
static void queueFragment(PLENTRY_INTERNAL* existingEntry, char* data, int offset, int length) {
    PLENTRY_INTERNAL entry;

    if (existingEntry == NULL || *existingEntry == NULL) {
        entry = (PLENTRY_INTERNAL)malloc(sizeof(*entry) + length);
    }
    else {
        entry = *existingEntry;
    }

    if (entry != NULL) {
        entry->entry.next = NULL;
        entry->entry.length = length;

        // If we had to allocate a new entry, we must copy the data. If not,
        // the data already resides within the LENTRY allocation.
        if (existingEntry == NULL || *existingEntry == NULL) {
            entry->allocPtr = entry;

            entry->entry.data = (char*)(entry + 1);
            memcpy(entry->entry.data, &data[offset], entry->entry.length);
        }
        else {
            entry->entry.data = &data[offset];

            // The caller should have already set this up for us
            LC_ASSERT(entry->allocPtr != NULL);

            // We now own the packet buffer and will manage freeing it
            *existingEntry = NULL;
        }

        entry->entry.bufferType = getBufferFlags(entry->entry.data, entry->entry.length);

        nalChainDataLength += entry->entry.length;

        if (nalChainTail == NULL) {
            LC_ASSERT(nalChainHead == NULL);
            nalChainHead = nalChainTail = (PLENTRY)entry;
        }
        else {
            LC_ASSERT(nalChainHead != NULL);
            nalChainTail->next = (PLENTRY)entry;
            nalChainTail = nalChainTail->next;
        }
    }
}

// Process an RTP Payload using the slow path that handles multiple NALUs per packet
static void processAvcHevcRtpPayloadSlow(PBUFFER_DESC currentPos, PLENTRY_INTERNAL* existingEntry) {
    // We should not have any NALUs when processing the first packet in an IDR frame
    LC_ASSERT(nalChainHead == NULL);
    LC_ASSERT(nalChainTail == NULL);

    while (currentPos->length != 0) {
        // Skip through any padding bytes
        if (!getAnnexBStartSequence(currentPos, NULL)) {
            skipToNextNal(currentPos);
        }

        // Skip any prepended AUD or SEI NALUs. We may have padding between
        // these on IDR frames, so the check in processRtpPayload() is not
        // completely sufficient to handle that case.
        while (isAccessUnitDelimiter(currentPos) || isSeiNal(currentPos)) {
            skipToNextNal(currentPos);
        }

        int start = currentPos->offset;
        bool containsPicData = false;

#ifdef FORCE_3_BYTE_START_SEQUENCES
        start++;
#endif

        if (isSeqReferenceFrameStart(currentPos)) {
            // No longer waiting for an IDR frame
            waitingForIdrFrame = false;
            waitingForRefInvalFrame = false;

            // Cancel any pending IDR frame request
            waitingForNextSuccessfulFrame = false;

            // Use the cached LENTRY for this NALU since it will be
            // the bulk of the data in this packet.
            containsPicData = true;

            // This is an IDR frame
            frameType = FRAME_TYPE_IDR;
        }

        // Move to the next NALU
        skipToNextNalOrEnd(currentPos);

        // If this is the picture data, we expect it to extend to the end of the packet
        if (containsPicData) {
            while (currentPos->length != 0) {
                // Any NALUs we encounter on the way to the end of the packet must be
                // reference frame slices or filler data.
                LC_ASSERT_VT(isSeqReferenceFrameStart(currentPos) || isFillerDataNal(currentPos));
                skipToNextNalOrEnd(currentPos);
            }
        }

        // To minimize copies, we'll allocate for SPS, PPS, and VPS to allow
        // us to reuse the packet buffer for the picture data in the I-frame.
        queueFragment(containsPicData ? existingEntry : NULL,
                      currentPos->data, start, currentPos->offset - start);
    }
}

// Dumps the decode unit queue and ensures the next frame submitted to the decoder will be
// an IDR frame
void requestDecoderRefresh(void) {
    // Wait for the next IDR frame
    waitingForIdrFrame = true;
    
    // Flush the decode unit queue
    freeDecodeUnitList(LbqFlushQueueItems(&decodeUnitQueue));
    
    // Request the receive thread drop its state
    // on the next call. We can't do it here because
    // it may be trying to queue DUs and we'll nuke
    // the state out from under it.
    dropStatePending = true;
    
    // Request the IDR frame
    LiRequestIdrFrame();
}

// Return 1 if packet is the first one in the frame
static bool isFirstPacket(uint8_t flags, uint8_t fecBlockNumber) {
    // Clear the picture data flag
    flags &= ~FLAG_CONTAINS_PIC_DATA;

    // Check if it's just the start or both start and end of a frame
    return (flags == (FLAG_SOF | FLAG_EOF) || flags == FLAG_SOF) && fecBlockNumber == 0;
}

// Process an RTP Payload
// The caller will free *existingEntry unless we NULL it
static void processRtpPayload(PNV_VIDEO_PACKET videoPacket, int length,
                       uint64_t receiveTimeMs, unsigned int presentationTimeMs,
                       PLENTRY_INTERNAL* existingEntry) {
    BUFFER_DESC currentPos;
    uint32_t frameIndex;
    uint8_t flags;
    bool firstPacket, lastPacket;
    uint32_t streamPacketIndex;
    uint8_t fecCurrentBlockNumber;
    uint8_t fecLastBlockNumber;

    // Mask the top 8 bits from the SPI
    videoPacket->streamPacketIndex >>= 8;
    videoPacket->streamPacketIndex &= 0xFFFFFF;

    currentPos.data = (char*)(videoPacket + 1);
    currentPos.offset = 0;
    currentPos.length = length - sizeof(*videoPacket);

    fecCurrentBlockNumber = (videoPacket->multiFecBlocks >> 4) & 0x3;
    fecLastBlockNumber = (videoPacket->multiFecBlocks >> 6) & 0x3;
    frameIndex = videoPacket->frameIndex;
    flags = videoPacket->flags;
    firstPacket = isFirstPacket(flags, fecCurrentBlockNumber);
    lastPacket = (flags & FLAG_EOF) && fecCurrentBlockNumber == fecLastBlockNumber;

    LC_ASSERT_VT((flags & ~(FLAG_SOF | FLAG_EOF | FLAG_CONTAINS_PIC_DATA)) == 0);

    streamPacketIndex = videoPacket->streamPacketIndex;
    
    // Drop packets from a previously corrupt frame
    if (isBefore32(frameIndex, nextFrameNumber)) {
        return;
    }

    // The FEC queue can sometimes recover corrupt frames (see comments in RtpFecQueue).
    // It almost always detects them before they get to us, but in case it doesn't
    // the streamPacketIndex not matching correctly should find nearly all of the rest.
    if (isBefore24(streamPacketIndex, U24(lastPacketInStream + 1)) ||
            (!(flags & FLAG_SOF) && streamPacketIndex != U24(lastPacketInStream + 1))) {
        Limelog("Depacketizer detected corrupt frame: %d", frameIndex);
        decodingFrame = false;
        nextFrameNumber = frameIndex + 1;
        dropFrameState();
        if (waitingForIdrFrame) {
            LiRequestIdrFrame();
        }
        else {
            reportFrameLoss(startFrameNumber, frameIndex);
        }
        return;
    }
    
    // Verify that we didn't receive an incomplete frame
    LC_ASSERT(firstPacket ^ decodingFrame);
    
    // Check sequencing of this frame to ensure we didn't
    // miss one in between
    if (firstPacket) {
        // Make sure this is the next consecutive frame
        if (isBefore32(nextFrameNumber, frameIndex)) {
            if (nextFrameNumber + 1 == frameIndex) {
                Limelog("Network dropped 1 frame (frame %d)\n", frameIndex - 1);
            }
            else {
                Limelog("Network dropped %d frames (frames %d to %d)\n",
                        frameIndex - nextFrameNumber,
                        nextFrameNumber,
                        frameIndex - 1);
            }

            // Fast ramp-up: loss on a wireless link usually precedes a jitter
            // burst that the percentile window has not seen yet.
            notifyJitterLoss();

            nextFrameNumber = frameIndex;

            // Wait until next complete frame
            waitingForNextSuccessfulFrame = true;
            dropFrameState();
        }
        else {
            LC_ASSERT(nextFrameNumber == frameIndex);
        }

        // We're now decoding a frame
        decodingFrame = true;
        frameType = FRAME_TYPE_PFRAME;
        firstPacketReceiveTime = receiveTimeMs;

        // Clear last frame's trace so a frame whose host extension is missing,
        // truncated or mismatched cannot inherit the previous frame's timestamps.
        frameTraceValid = false;
        frameTraceLastPacketRxUs = 0;
        frameTraceLastPacketRxValid = false;
        
        // Some versions of Sunshine don't send a valid PTS, so we will
        // synthesize one using the receive time as the time base.
        if (!syntheticPtsBase) {
            syntheticPtsBase = receiveTimeMs;
        }
        
        if (!presentationTimeMs && frameIndex > 0) {
            firstPacketPresentationTime = (unsigned int)(receiveTimeMs - syntheticPtsBase);
        }
        else {
            firstPacketPresentationTime = presentationTimeMs;
        }
    }

    // SPEC.md §3 t_last_packet_rx.
    //
    // This MUST be sampled after the firstPacket block above, not before it. A
    // frame small enough to fit in one RTP packet carries FLAG_SOF and FLAG_EOF
    // together, so firstPacket and lastPacket are both true in this single
    // invocation; sampling first would have the SOF reset immediately zero it.
    // Single-packet P-frames are common on a static screen, so that would have
    // silently blanked a large fraction of rows.
    //
    // Validity is carried explicitly rather than by testing for zero, because a
    // zero here is indistinguishable from a real value to the consumer.
    if (LatencyTraceEnabled && lastPacket) {
        frameTraceLastPacketRxUs = PltGetMicros();
        frameTraceLastPacketRxValid = true;
    }

    lastPacketInStream = streamPacketIndex;

    // If this is the first packet, skip the frame header (if one exists)
    uint32_t frameHeaderSize;
    LC_ASSERT_VT(currentPos.length > 0);
    if (firstPacket && currentPos.length > 0) {
        // Parse the frame type from the header
        LC_ASSERT_VT(currentPos.length >= 4);
        if (APP_VERSION_AT_LEAST(7, 1, 350) && currentPos.length >= 4) {
            switch (currentPos.data[currentPos.offset + 3]) {
            case 1: // Normal P-frame
                break;
            case 2: // IDR frame
                // For other codecs, we trust the frame header rather than parsing the bitstream
                // to determine if a given frame is an IDR frame.
                if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265))) {
                    waitingForIdrFrame = false;
                    waitingForNextSuccessfulFrame = false;
                    frameType = FRAME_TYPE_IDR;
                }
                // Fall-through
            case 4: // Intra-refresh
            case 5: // P-frame with reference frames invalidated
                if (waitingForRefInvalFrame) {
                    Limelog("Next post-invalidation frame is: %d (%s-frame)\n",
                            frameIndex,
                            currentPos.data[currentPos.offset + 3] == 5 ? "P" : "I");
                    waitingForRefInvalFrame = false;
                    waitingForNextSuccessfulFrame = false;
                }
                break;
            case 104: // Sunshine hardcoded header
                break;
            default:
                Limelog("Unrecognized frame type: %d", currentPos.data[currentPos.offset + 3]);
                LC_ASSERT_VT(false);
                break;
            }
        }
        else {
            // Hope for the best with older servers
            if (waitingForRefInvalFrame) {
                reportFrameLoss(startFrameNumber, frameIndex - 1);
                waitingForRefInvalFrame = false;
                waitingForNextSuccessfulFrame = false;
            }
        }

        // Sunshine can provide host processing latency of the frame
        LC_ASSERT_VT(currentPos.length >= 3);
        if (IS_SUNSHINE() && currentPos.length >= 3) {
            BYTE_BUFFER bb;
            BbInitializeWrappedBuffer(&bb, currentPos.data, currentPos.offset + 1, 2, BYTE_ORDER_LITTLE);
            BbGet16(&bb, &frameHostProcessingLatency);
        }

        // Codecs like H.264 and HEVC handle the FEC trailing zero padding just fine, but other
        // codecs need the exact length encoded separately.
        LC_ASSERT_VT(currentPos.length >= 6);
        if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) && currentPos.length >= 6) {
            BYTE_BUFFER bb;
            BbInitializeWrappedBuffer(&bb, currentPos.data, currentPos.offset + 4, 2, BYTE_ORDER_LITTLE);
            BbGet16(&bb, &lastPacketPayloadLength);
        }

        // The Apollo 2.0 latency trace appends a SS_FRAME_TIMESTAMP_EXT to the
        // stock frame header and signals it with the stock discriminator plus
        // one (0x01 -> 0x02, 0x81 -> 0x82). Subtract to recover the stock value
        // so the version cascade below is unchanged, then skip the extension
        // after the stock header size has been determined.
        //
        // The extension is always skipped correctly if it is present, even when
        // the trace is disabled locally. Only reading the timestamps is gated on
        // LatencyTraceEnabled. That way a host that keeps emitting the extension
        // after a mid-session capability change degrades to "no trace data"
        // instead of desynchronising the bitstream.
        uint8_t frameHdrDisc = (uint8_t)currentPos.data[0];
        bool frameHdrHasTraceExt = false;
        if (frameHdrDisc == FRAME_HDR_DISC_SHORT_TRACE || frameHdrDisc == FRAME_HDR_DISC_LONG_TRACE) {
            // 0x02 -> 0x01 and 0x82 -> 0x81.
            frameHdrHasTraceExt = true;
            frameHdrDisc -= 1;
        }

        if (APP_VERSION_AT_LEAST(7, 1, 450)) {
            // >= 7.1.450 uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 44 byte header
            if (frameHdrDisc == FRAME_HDR_DISC_SHORT) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(frameHdrDisc == FRAME_HDR_DISC_LONG);
                frameHeaderSize = 44;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 446)) {
            // [7.1.446, 7.1.450) uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 41 byte header
            if (frameHdrDisc == FRAME_HDR_DISC_SHORT) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(frameHdrDisc == FRAME_HDR_DISC_LONG);
                frameHeaderSize = 41;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 415)) {
            // [7.1.415, 7.1.446) uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 24 byte header
            if (frameHdrDisc == FRAME_HDR_DISC_SHORT) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(frameHdrDisc == FRAME_HDR_DISC_LONG);
                frameHeaderSize = 24;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 350)) {
            // [7.1.350, 7.1.415) should use the 8 byte header again
            frameHeaderSize = 8;
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 320)) {
            // [7.1.320, 7.1.350) should use the 12 byte frame header
            frameHeaderSize = 12;
        }
        else if (APP_VERSION_AT_LEAST(5, 0, 0)) {
            // [5.x, 7.1.320) should use the 8 byte header
            frameHeaderSize = 8;
        }
        else {
            // Other versions don't have a frame header at all
            frameHeaderSize = 0;
        }

        // Parse the latency trace extension, which sits between the stock frame
        // header and the picture data. Any validation failure means we skip the
        // trace for this frame but still consume the bytes, so the bitstream
        // stays intact.
        if (frameHdrHasTraceExt) {
            if (currentPos.length < frameHeaderSize + sizeof(SS_FRAME_TIMESTAMP_EXT)) {
                // The discriminator promised an extension that does not fit. We
                // cannot locate the picture data, so the frame is unusable.
                //
                // Note the length assert further down does NOT catch this: it
                // only checks against the stock header size, which any length
                // between stock and stock+47 satisfies. Dropping the frame state
                // explicitly is the actual handling.
                Limelog("Frame %d: truncated latency trace extension (%u < %u); dropping frame\n",
                        frameIndex, currentPos.length,
                        (unsigned int)(frameHeaderSize + sizeof(SS_FRAME_TIMESTAMP_EXT)));
                frameTraceValid = false;
                dropFrameState();
                return;
            }
            else {
                if (LatencyTraceEnabled) {
                    SS_FRAME_TIMESTAMP_EXT ext;

                    // memcpy because the extension is not guaranteed to be aligned
                    // within the packet buffer.
                    memcpy(&ext, currentPos.data + currentPos.offset + frameHeaderSize, sizeof(ext));

                    ext.frameIndex = LE32(ext.frameIndex);
                    ext.captureRequestedUs = LE64(ext.captureRequestedUs);
                    ext.captureCompleteUs = LE64(ext.captureCompleteUs);
                    ext.encodeSubmitUs = LE64(ext.encodeSubmitUs);
                    ext.encodeCompleteUs = LE64(ext.encodeCompleteUs);
                    ext.txPipelineEntryUs = LE64(ext.txPipelineEntryUs);

                    if (ext.extVersion == 0 || ext.extVersion > SS_FRAME_TIMESTAMP_EXT_VERSION) {
                        // A host speaking a version we do not understand. Ignore
                        // the contents rather than misinterpreting them; the
                        // struct size is fixed by contract so the picture data is
                        // still located correctly below.
                        //
                        // This used to be silent, which made a version mismatch
                        // present as "every host column is empty" with no
                        // diagnostic anywhere on either side. Log it once per
                        // session rather than per frame.
                        if (!frameTraceVersionMismatchLogged) {
                            frameTraceVersionMismatchLogged = true;
                            Limelog("Latency trace: host is emitting frame timestamp extension v%u "
                                    "but this client understands at most v%u; host columns will be "
                                    "empty for this session\n",
                                    ext.extVersion, SS_FRAME_TIMESTAMP_EXT_VERSION);
                        }
                        frameTraceValid = false;
                    }
                    else if (ext.frameIndex != frameIndex) {
                        // The echo did not match, so this extension does not belong
                        // to this frame. Never emit a row joined on the wrong id.
                        Limelog("Frame %d: latency trace extension frame index mismatch (%u)\n",
                                frameIndex, ext.frameIndex);
                        frameTraceValid = false;
                    }
                    else {
                        // v1 predates validityMask and always sent that byte as
                        // zero, so an unpatched host would otherwise look like it
                        // had no stamps at all. v1 implicitly means all five are
                        // present, which is what v1 actually guaranteed.
                        if (ext.extVersion < 2) {
                            ext.validityMask = SS_STAMP_VALID_ALL;
                        }
                        ext.validityMask &= SS_STAMP_VALID_ALL;

                        frameTraceExt = ext;
                        frameTraceValid = true;

                        if (!frameTraceVersionLogged) {
                            frameTraceVersionLogged = true;
                            frameTraceNegotiatedVersion = ext.extVersion;
                            Limelog("Latency trace: host is emitting frame timestamp extension v%u\n",
                                    ext.extVersion);
                        }
                    }
                }

                frameHeaderSize += (uint32_t)sizeof(SS_FRAME_TIMESTAMP_EXT);
            }
        }

        LC_ASSERT_VT(currentPos.length >= frameHeaderSize);
        if (currentPos.length >= frameHeaderSize) {
            // Skip past the frame header
            currentPos.offset += frameHeaderSize;
            currentPos.length -= frameHeaderSize;
        }

        // We only parse H.264 and HEVC at the NALU level
        if (NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) {
            // The Annex B NALU start prefix must be next
            if (!getAnnexBStartSequence(&currentPos, NULL)) {
                // If we aren't starting on a start prefix, something went wrong.
                LC_ASSERT_VT(false);

                // For release builds, we will try to recover by searching for one.
                // This mimics the way most decoders handle this situation.
                skipToNextNal(&currentPos);
            }

            // If an AUD NAL is prepended to this frame data, remove it.
            // Other parts of this code are not prepared to deal with a
            // NAL of that type, so stripping it is the easiest option.
            if (isAccessUnitDelimiter(&currentPos)) {
                skipToNextNal(&currentPos);
            }

            // There may be one or more SEI NAL units prepended to the
            // frame data *after* the (optional) AUD.
            while (isSeiNal(&currentPos)) {
                skipToNextNal(&currentPos);
            }
        }
    }
    else {
        // There is no frame header on later packets
        frameHeaderSize = 0;
    }

    if (NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) {
        if (firstPacket && isIdrFrameStart(&currentPos)) {
            // SPS and PPS prefix is padded between NALs, so we must decode it with the slow path
            processAvcHevcRtpPayloadSlow(&currentPos, existingEntry);
        }
        else {
            // Intel's H.264 Media Foundation encoder prepends a PPS to each P-frame.
            // Skip it to avoid confusing clients.
            if (firstPacket && isPictureParameterSetNal(&currentPos)) {
                skipToNextNal(&currentPos);
            }

#ifdef FORCE_3_BYTE_START_SEQUENCES
            if (firstPacket) {
                currentPos.offset++;
                currentPos.length--;
            }
#endif

            queueFragment(existingEntry, currentPos.data, currentPos.offset, currentPos.length);
        }
    }
    else {
        // We fixup the length of the last packet for other codecs since they may not be tolerant
        // of trailing zero padding like H.264/HEVC Annex B bitstream parsers are.
        if (lastPacket) {
            // The payload length includes the frame header, so it cannot be smaller than that
            LC_ASSERT_VT(lastPacketPayloadLength > frameHeaderSize);

            // The payload length cannot be smaller than the actual received payload
            // NB: currentPos.length is already adjusted to exclude the frameHeaderSize from above
            LC_ASSERT_VT(lastPacketPayloadLength - frameHeaderSize <= currentPos.length);

            // If the payload length is valid, truncate the packet. If not, discard this frame.
            if (lastPacketPayloadLength > frameHeaderSize && lastPacketPayloadLength - frameHeaderSize <= currentPos.length) {
                currentPos.length = lastPacketPayloadLength - frameHeaderSize;
            }
            else {
                if (lastPacketPayloadLength <= frameHeaderSize) {
                    Limelog("Invalid last payload length for header on frame %u: %u <= %u",
                            frameIndex, lastPacketPayloadLength, frameHeaderSize);
                }
                else {
                    Limelog("Invalid last payload length for packet size on frame %u: %u > %u",
                            frameIndex, lastPacketPayloadLength - frameHeaderSize, currentPos.length);
                }

                // Skip to the next frame and tell the host we lost this one
                decodingFrame = false;
                nextFrameNumber = frameIndex + 1;
                dropFrameState();
                if (waitingForIdrFrame) {
                    LiRequestIdrFrame();
                }
                else {
                    reportFrameLoss(startFrameNumber, frameIndex);
                }

                return;
            }
        }

        // Other codecs are just passed through as is.
        queueFragment(existingEntry, currentPos.data, currentPos.offset, currentPos.length);
    }

    if (lastPacket) {
        // Move on to the next frame
        decodingFrame = false;
        nextFrameNumber = frameIndex + 1;

        // If we can't submit this frame due to a discontinuity in the bitstream,
        // inform the host (if needed) and drop the data.
        if (waitingForIdrFrame || waitingForRefInvalFrame) {
            // IDR wait takes priority over RFI wait (and an IDR frame will satisfy both)
            if (waitingForIdrFrame) {
                Limelog("Waiting for IDR frame\n");

                // We wait for the first fully received frame after a loss to approximate
                // detection of the recovery of the network. Requesting an IDR frame while
                // the network is unstable will just contribute to congestion collapse.
                if (waitingForNextSuccessfulFrame) {
                    LiRequestIdrFrame();
                }
            }
            else {
                // If we need an RFI frame first, then drop this frame
                // and update the reference frame invalidation window.
                Limelog("Waiting for RFI frame\n");
                reportFrameLoss(startFrameNumber, frameIndex);
            }

            waitingForNextSuccessfulFrame = false;
            dropFrameState();
            return;
        }

        LC_ASSERT(!waitingForNextSuccessfulFrame);

        // Carry out any pending state drops. We can't just do this
        // arbitrarily in the middle of processing a frame because
        // may cause the depacketizer state to become corrupted. For
        // example, if we drop state after the first packet, the
        // depacketizer will next try to process a non-SOF packet,
        // and cause it to assert.
        if (dropStatePending) {
            if (nalChainHead && frameType == FRAME_TYPE_IDR) {
                // Don't drop the frame state if this frame is an IDR frame itself,
                // otherwise we'll lose this IDR frame without another in flight
                // and have to wait until we hit our consecutive drop limit to
                // request a new one (potentially several seconds).
                dropStatePending = false;
            }
            else {
                dropFrameState();
                return;
            }
        }

        reassembleFrame(frameIndex);
    }
}

// Called by the video RTP FEC queue to notify us of a lost frame
// if it determines the frame to be unrecoverable. This lets us
// avoid having to wait until the next received frame to determine
// that we lost a frame and submit an RFI request.
void notifyFrameLost(unsigned int frameNumber, bool speculative) {
    // We may not invalidate frames that we've already received
    LC_ASSERT(frameNumber >= startFrameNumber);

    // Drop state and determine if we need an IDR frame or if RFI is okay
    dropFrameState();

    // If dropFrameState() determined that RFI was usable, issue it now
    if (!waitingForIdrFrame) {
        LC_ASSERT(waitingForRefInvalFrame);

        if (speculative) {
            Limelog("Sending speculative RFI request for predicted loss of frame %d\n", frameNumber);
        }
        else {
            Limelog("Sending RFI request for unrecoverable frame %d\n", frameNumber);
        }

        // Advance the frame number since we won't be expecting this one anymore
        nextFrameNumber = frameNumber + 1;

        // Notify the host that we lost this one
        reportFrameLoss(startFrameNumber, frameNumber);
    }
}

// Add an RTP Packet to the queue
void queueRtpPacket(PRTPV_QUEUE_ENTRY queueEntryPtr) {
    int dataOffset;
    RTPV_QUEUE_ENTRY queueEntry = *queueEntryPtr;

    LC_ASSERT(!queueEntry.isParity);
    LC_ASSERT(queueEntry.receiveTimeMs != 0);

    dataOffset = sizeof(*queueEntry.packet);
    if (queueEntry.packet->header & FLAG_EXTENSION) {
        dataOffset += 4; // 2 additional fields
    }

    // The packet length was validated by the RtpVideoQueue
    LC_ASSERT(queueEntry.length >= dataOffset + (int)sizeof(NV_VIDEO_PACKET));

    // Reuse the memory reserved for the RTPFEC_QUEUE_ENTRY to store the LENTRY_INTERNAL
    // now that we're in the depacketizer. We saved a copy of the real FEC queue entry
    // on the stack here so we can safely modify this memory in place.
    LC_ASSERT(sizeof(LENTRY_INTERNAL) <= sizeof(RTPV_QUEUE_ENTRY));
    PLENTRY_INTERNAL existingEntry = (PLENTRY_INTERNAL)queueEntryPtr;
    existingEntry->allocPtr = queueEntry.packet;

    processRtpPayload((PNV_VIDEO_PACKET)(((char*)queueEntry.packet) + dataOffset),
                      queueEntry.length - dataOffset,
                      queueEntry.receiveTimeMs,
                      queueEntry.presentationTimeMs,
                      &existingEntry);

    if (existingEntry != NULL) {
        // processRtpPayload didn't want this packet, so just free it
        free(existingEntry->allocPtr);
    }
}

int LiGetPendingVideoFrames(void) {
    return LbqGetItemCount(&decodeUnitQueue);
}
