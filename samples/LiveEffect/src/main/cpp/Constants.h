//
// Created by Shariq Mobin on 2019-06-17.
//

#ifndef SAMPLES_CONSTANTS_H
#define SAMPLES_CONSTANTS_H

constexpr int32_t kStreamSampleRate = 48000; // For now we force this so we can resample easily.
constexpr int32_t kModelSampleRate = 8000; // Fixed sample rate, see README

constexpr int kBufferSizeInBursts = 2; // Use 2 bursts as the buffer size (double buffer)
// XXX(shariq): Changed to 4 because: BurstSize = 96. TFLite wants 128 chunks. 4 * 96 = 3 * 128
//constexpr int kBufferSizeInBursts = 8; // Use 4 bursts as the buffer size (quadruple buffer)


// constexpr int kMaxQueueItems = 8192; // Must be power of 2
constexpr int kMaxQueueItems = 32768; // Must be power of 2

#endif //SAMPLES_CONSTANTS_H
