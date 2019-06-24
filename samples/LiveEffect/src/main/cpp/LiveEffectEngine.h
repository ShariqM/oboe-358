/*
 * Copyright 2018 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef OBOE_LIVEEFFECTENGINE_H
#define OBOE_LIVEEFFECTENGINE_H

#include <jni.h>
#include <oboe/Oboe.h>
#include <string>
#include <thread>
#include <future>
#include "FullDuplexPass.h"
#include "LockFreeQueue.h"
#include "Constants.h"

class LiveEffectEngine : public oboe::AudioStreamCallback {
   public:
    LiveEffectEngine();
    ~LiveEffectEngine();
    void setRecordingDeviceId(int32_t deviceId);
    void setPlaybackDeviceId(int32_t deviceId);
    void setEffectOn(bool isOn);
    void openStreams();

    /*
     * oboe::AudioStreamCallback interface implementation
     */
    oboe::DataCallbackResult onAudioReady(oboe::AudioStream *oboeStream,
                                          void *audioData, int32_t numFrames);
    void onErrorBeforeClose(oboe::AudioStream *oboeStream, oboe::Result error);
    void onErrorAfterClose(oboe::AudioStream *oboeStream, oboe::Result error);

    bool setAudioApi(oboe::AudioApi);
    bool isAAudioSupported(void);

   private:
    const char* TAG = "LiveEffectEngine";
    FullDuplexPass mFullDuplexPass;

    // TODO add getters and setters
    static constexpr int32_t kNumCallbacksToDrain   = 1;
    //static constexpr int32_t kNumCallbacksToDiscard = 30;

    // let input fill back up, usually 0 or 1
    int32_t              mNumInputBurstsCushion = 1;

    // We want to reach a state where the input buffer is empty and
    // the output buffer is full.
    // These are used in order.
    // Drain several callback so that input is empty.
    int32_t              mCountCallbacksToDrain = kNumCallbacksToDrain;
    // Let the input fill back up slightly so we don't run dry.
    int32_t              mCountInputBurstsCushion = mNumInputBurstsCushion;
    // Discard some callbacks so the input and output reach equilibrium.
    //int32_t              mCountCallbacksToDiscard = kNumCallbacksToDiscard;

    LockFreeQueue<int16_t, kMaxQueueItems> queue;
    void drainQueue();

    bool recordingDrained = false;
    bool mIsEffectOn = false;
    int32_t mRecordingDeviceId = oboe::kUnspecified;
    int32_t mPlaybackDeviceId = oboe::kUnspecified;
    oboe::AudioFormat mFormat = oboe::AudioFormat::I16;
    int32_t mSampleRate = oboe::kUnspecified;
    int32_t mInputChannelCount = oboe::ChannelCount::Mono;
    int32_t mOutputChannelCount = oboe::ChannelCount::Mono;
    oboe::AudioStream *mRecordingStream = nullptr;
    oboe::AudioStream *mPlayStream = nullptr;
    oboe::AudioApi mAudioApi = oboe::AudioApi::AAudio;

    void closeStream(oboe::AudioStream *stream);

    void recordAndWrite(std::future<void> exitFuture);
    std::thread createRecordAndWriteThread();
    std::thread recordAndWriteThread;
    std::promise<void> threadExitSignal;




    oboe::AudioStreamBuilder *setupCommonStreamParameters(
        oboe::AudioStreamBuilder *builder);
    oboe::AudioStreamBuilder *setupRecordingStreamParameters(
        oboe::AudioStreamBuilder *builder);
    oboe::AudioStreamBuilder *setupPlaybackStreamParameters(
        oboe::AudioStreamBuilder *builder);
    void warnIfNotLowLatency(oboe::AudioStream *stream);
};

#endif  // OBOE_LIVEEFFECTENGINE_H
