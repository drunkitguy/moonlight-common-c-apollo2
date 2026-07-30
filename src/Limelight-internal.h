#pragma once

#include "Platform.h"
#include "Limelight.h"
#include "PlatformSockets.h"
#include "PlatformThreads.h"
#include "PlatformCrypto.h"
#include "Video.h"
#include "Input.h"
#include "RtpAudioQueue.h"
#include "RtpVideoQueue.h"
#include "ByteBuffer.h"

#include <enet/enet.h>

// Common globals
extern char* RemoteAddrString;
extern struct sockaddr_storage RemoteAddr;
extern struct sockaddr_storage LocalAddr;
extern SOCKADDR_LEN AddrLen;
extern int AppVersionQuad[4];
extern STREAM_CONFIGURATION StreamConfig;
extern CONNECTION_LISTENER_CALLBACKS ListenerCallbacks;
extern DECODER_RENDERER_CALLBACKS VideoCallbacks;
extern AUDIO_RENDERER_CALLBACKS AudioCallbacks;
extern int NegotiatedVideoFormat;
extern volatile bool ConnectionInterrupted;
extern bool HighQualitySurroundSupported;
extern bool HighQualitySurroundEnabled;
extern OPUS_MULTISTREAM_CONFIGURATION NormalQualityOpusConfig;
extern OPUS_MULTISTREAM_CONFIGURATION HighQualityOpusConfig;
extern int AudioPacketDuration;
extern bool AudioEncryptionEnabled;
extern bool ReferenceFrameInvalidationSupported;

extern uint16_t RtspPortNumber;
extern uint16_t ControlPortNumber;
extern uint16_t AudioPortNumber;
extern uint16_t VideoPortNumber;

extern SS_PING AudioPingPayload;
extern SS_PING VideoPingPayload;
extern uint32_t ControlConnectData;

extern uint32_t SunshineFeatureFlags;

// Highest frame timestamp extension version the host advertised in DESCRIBE
// (x-ss-general.traceExtVersion), or 0 if it advertised none. Diagnostic only:
// the authoritative version is the one on each extension. Written once during
// the RTSP handshake, before any thread that reads it starts.
extern uint32_t HostFrameTraceExtVersion;

// Encryption flags shared by Sunshine and Moonlight in RTSP
#define SS_ENC_CONTROL_V2 0x01
#define SS_ENC_VIDEO 0x02
#define SS_ENC_AUDIO 0x04

extern uint32_t EncryptionFeaturesSupported;
extern uint32_t EncryptionFeaturesRequested;
extern uint32_t EncryptionFeaturesEnabled;

// ENet channel ID values
#define CTRL_CHANNEL_GENERIC      0x00
#define CTRL_CHANNEL_URGENT       0x01 // IDR and reference frame invalidation requests
#define CTRL_CHANNEL_KEYBOARD     0x02
#define CTRL_CHANNEL_MOUSE        0x03
#define CTRL_CHANNEL_PEN          0x04
#define CTRL_CHANNEL_TOUCH        0x05
#define CTRL_CHANNEL_UTF8         0x06
#define CTRL_CHANNEL_SERVERCTL    0x08
#define CTRL_CHANNEL_GAMEPAD_BASE 0x10 // 0x10 to 0x1F by controller index
#define CTRL_CHANNEL_SENSOR_BASE  0x20 // 0x20 to 0x2F by controller index
#define CTRL_CHANNEL_COUNT        0x30

#ifndef UINT24_MAX
#define UINT24_MAX 0xFFFFFF
#endif

#define U16(x) ((unsigned short) ((x) & UINT16_MAX))
#define U24(x) ((unsigned int) ((x) & UINT24_MAX))
#define U32(x) ((unsigned int) ((x) & UINT32_MAX))

#define isBefore16(x, y) (U16((x) - (y)) > (UINT16_MAX/2))
#define isBefore24(x, y) (U24((x) - (y)) > (UINT24_MAX/2))
#define isBefore32(x, y) (U32((x) - (y)) > (UINT32_MAX/2))

#define APP_VERSION_AT_LEAST(a, b, c)                                                       \
    ((AppVersionQuad[0] > (a)) ||                                                           \
     (AppVersionQuad[0] == (a) && AppVersionQuad[1] > (b)) ||                               \
     (AppVersionQuad[0] == (a) && AppVersionQuad[1] == (b) && AppVersionQuad[2] >= (c)))

#define IS_SUNSHINE() (AppVersionQuad[3] < 0)

// Client feature flags for x-ml-general.featureFlags SDP attribute
#define ML_FF_FEC_STATUS 0x01 // Client sends SS_FRAME_FEC_STATUS for frame losses
#define ML_FF_SESSION_ID_V1 0x02 // Client supports X-SS-Ping-Payload and X-SS-Connect-Data
#define ML_FF_LATENCY_TRACE 0x04 // Client understands SS_FRAME_TIMESTAMP_EXT and the clock sync messages

// Host feature flags in x-ss-general.featureFlags. These must match the host's
// platform_caps constants (Apollo: src/platform/common.h).
#define SS_FF_PEN_TOUCH_EVENTS 0x01
#define SS_FF_CONTROLLER_TOUCH_EVENTS 0x02
#define SS_FF_LATENCY_TRACE 0x04 // Host emits SS_FRAME_TIMESTAMP_EXT and answers clock sync

// True only when BOTH peers advertised the latency trace capability and the
// client enabled it in STREAM_CONFIGURATION. Read on the video receive thread
// and the control threads; written once during connection setup before any of
// those threads start, so it needs no synchronisation after startup.
extern bool LatencyTraceEnabled;

#define UDP_RECV_POLL_TIMEOUT_MS 100

// At this value or above, we will request high quality audio unless CAPABILITY_SLOW_OPUS_DECODER
// is set on the audio renderer.
#define HIGH_AUDIO_BITRATE_THRESHOLD 15000

// Below this value, we will request 20 ms audio frames to reduce bandwidth if the audio
// renderer sets CAPABILITY_SUPPORTS_ARBITRARY_AUDIO_DURATION.
#define LOW_AUDIO_BITRATE_TRESHOLD 5000

// Internal macro for checking the magic byte of the audio configuration value
#define MAGIC_BYTE_FROM_AUDIO_CONFIG(x) ((x) & 0xFF)

int serviceEnetHost(ENetHost* client, ENetEvent* event, enet_uint32 timeoutMs);
int gracefullyDisconnectEnetPeer(ENetHost* host, ENetPeer* peer, enet_uint32 lingerTimeoutMs);
int extractVersionQuadFromString(const char* string, int* quad);
bool isReferenceFrameInvalidationSupportedByDecoder(void);
bool isReferenceFrameInvalidationEnabled(void);
void* extendBuffer(void* ptr, size_t newSize);

void fixupMissingCallbacks(PDECODER_RENDERER_CALLBACKS* drCallbacks, PAUDIO_RENDERER_CALLBACKS* arCallbacks,
    PCONNECTION_LISTENER_CALLBACKS* clCallbacks);
void setRecorderCallbacks(PDECODER_RENDERER_CALLBACKS drCallbacks, PAUDIO_RENDERER_CALLBACKS arCallbacks);

char* getSdpPayloadForStreamConfig(int rtspClientVersion, int* length);

int initializeControlStream(void);
int startControlStream(void);
int stopControlStream(void);
void destroyControlStream(void);
void connectionDetectedFrameLoss(uint32_t startFrame, uint32_t endFrame);
void connectionReceivedCompleteFrame(uint32_t frameIndex);
void connectionSawFrame(uint32_t frameIndex);
void connectionSendFrameFecStatus(PSS_FRAME_FEC_STATUS fecStatus);
int sendInputPacketOnControlStream(unsigned char* data, int length, uint8_t channelId, uint32_t flags, bool moreData);
void flushInputOnControlStream(void);
bool isControlDataInTransit(void);

// Converts a host CLOCK_MONOTONIC timestamp to the client's monotonic epoch
// using the current clock offset estimate. Returns false when no valid estimate
// exists yet, when the estimate has been invalidated by divergence, or when the
// conversion would underflow. Callers must drop the sample when this returns
// false rather than emitting a garbage value.
bool convertHostToClientMicros(uint64_t hostUs, uint64_t* clientUs);

// Frame timestamp extension version observed on the wire this session, 0 if none.
// Video receive thread writes it once; read at session end for the trace metadata.
uint8_t getFrameTraceExtVersion(void);

int performRtspHandshake(PSERVER_INFORMATION serverInfo);

void initializeVideoDepacketizer(int pktSize);
void destroyVideoDepacketizer(void);
void queueRtpPacket(PRTPV_QUEUE_ENTRY queueEntry);
void stopVideoDepacketizer(void);
void requestDecoderRefresh(void);
void notifyFrameLost(unsigned int frameNumber, bool speculative);

void initializeVideoStream(void);
void destroyVideoStream(void);
void notifyKeyFrameReceived(void);
int startVideoStream(void* rendererContext, int drFlags);
void stopVideoStream(void);

int initializeAudioStream(void);
int notifyAudioPortNegotiationComplete(void);
void destroyAudioStream(void);
int startAudioStream(void* audioContext, int arFlags);
void stopAudioStream(void);

int initializeInputStream(void);
void destroyInputStream(void);
int startInputStream(void);
int stopInputStream(void);
