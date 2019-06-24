/**
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

#include "LiveEffectEngine.h"
#include <assert.h>
#include <logging_macros.h>


LiveEffectEngine::LiveEffectEngine() {
    assert(mOutputChannelCount == mInputChannelCount);
}

LiveEffectEngine::~LiveEffectEngine() {

    mFullDuplexPass.stop();
    closeStream(mPlayStream);
    // Thread Safety
    threadExitSignal.set_value();  // Tell the thread to stop reading from the recordingStream.
    recordAndWriteThread.join();  // Wait until it's done.
    drainQueue();  // No one is touching the queue now.
    closeStream(mRecordingStream);

}

void LiveEffectEngine::setRecordingDeviceId(int32_t deviceId) {
    mRecordingDeviceId = deviceId;
}

void LiveEffectEngine::setPlaybackDeviceId(int32_t deviceId) {
    mPlaybackDeviceId = deviceId;
}

bool LiveEffectEngine::isAAudioSupported() {
    oboe::AudioStreamBuilder builder;
    return builder.isAAudioSupported();
}
bool LiveEffectEngine::setAudioApi(oboe::AudioApi api) {
    if (mIsEffectOn) return false;

    mAudioApi = api;
    return true;
}
void LiveEffectEngine::setEffectOn(bool isOn) {
    if (isOn != mIsEffectOn) {
        mIsEffectOn = isOn;
        if (isOn) {
            openStreams();
            mSampleRate = mPlayStream->getSampleRate();
            mFullDuplexPass.start();

            // Reset startup variables.
            recordingDrained = false;
            mCountInputBurstsCushion = mNumInputBurstsCushion;
            mCountCallbacksToDrain = kNumCallbacksToDrain;

            recordAndWriteThread = createRecordAndWriteThread();
        } else {
            mFullDuplexPass.stop();
            /*
            * Note: The order of events is important here.
            * The playback stream must be closed before the recording stream. If the
            * recording stream were to be closed first the playback stream's
            * callback may attempt to read from the recording stream
            * which would cause the app to crash since the recording stream would be
            * null.
            */

            closeStream(mPlayStream);
            // Thread Safety
            threadExitSignal.set_value();  // Tell the thread to stop reading from the recordingStream.
            recordAndWriteThread.join();  // Wait until it's done.
            drainQueue();  // No one is touching the queue now.
            closeStream(mRecordingStream);
       }
    }
}

void LiveEffectEngine::drainQueue() {
    int16_t value;
    while (queue.pop(value));
}

void LiveEffectEngine::openStreams() {
    // Note: The order of stream creation is important. We create the playback
    // stream first, then use properties from the playback stream
    // (e.g. sample rate) to create the recording stream. By matching the
    // properties we should get the lowest latency path
    oboe::AudioStreamBuilder inBuilder, outBuilder;
    setupPlaybackStreamParameters(&outBuilder);
    outBuilder.openStream(&mPlayStream);
    warnIfNotLowLatency(mPlayStream);

    setupRecordingStreamParameters(&inBuilder);
    inBuilder.openStream(&mRecordingStream);
    warnIfNotLowLatency(mRecordingStream);

    mFullDuplexPass.setInputStream(mRecordingStream);
    mFullDuplexPass.setOutputStream(mPlayStream);
}

/**
 * Sets the stream parameters which are specific to recording,
 * including the sample rate which is determined from the
 * playback stream.
 *
 * @param builder The recording stream builder
 */
oboe::AudioStreamBuilder *LiveEffectEngine::setupRecordingStreamParameters(
    oboe::AudioStreamBuilder *builder) {
    // This sample uses blocking read() by setting callback to null
    builder->setCallback(nullptr)
        ->setDeviceId(mRecordingDeviceId)
        ->setDirection(oboe::Direction::Input)
        ->setSampleRate(mSampleRate)
        ->setChannelCount(mInputChannelCount);
    return setupCommonStreamParameters(builder);
}

/**
 * Sets the stream parameters which are specific to playback, including device
 * id and the dataCallback function, which must be set for low latency
 * playback.
 * @param builder The playback stream builder
 */
oboe::AudioStreamBuilder *LiveEffectEngine::setupPlaybackStreamParameters(
    oboe::AudioStreamBuilder *builder) {
    builder->setCallback(this)
        ->setDeviceId(mPlaybackDeviceId)
        ->setDirection(oboe::Direction::Output)
        ->setChannelCount(mOutputChannelCount);

    return setupCommonStreamParameters(builder);
}

/**
 * Set the stream parameters which are common to both recording and playback
 * streams.
 * @param builder The playback or recording stream builder
 */
oboe::AudioStreamBuilder *LiveEffectEngine::setupCommonStreamParameters(
    oboe::AudioStreamBuilder *builder) {
    // We request EXCLUSIVE mode since this will give us the lowest possible
    // latency.
    // If EXCLUSIVE mode isn't available the builder will fall back to SHARED
    // mode.
    builder->setAudioApi(mAudioApi)
        ->setFormat(mFormat)
        ->setSharingMode(oboe::SharingMode::Exclusive)
        ->setPerformanceMode(oboe::PerformanceMode::LowLatency);
    return builder;
}


/**
 * Close the stream. AudioStream::close() is a blocking call so
 * the application does not need to add synchronization between
 * onAudioReady() function and the thread calling close().
 * [the closing thread is the UI thread in this sample].
 * @param stream the stream to close
 */
void LiveEffectEngine::closeStream(oboe::AudioStream *stream) {
    if (stream) {
        oboe::Result result = stream->close();
        if (result != oboe::Result::OK) {
            LOGE("Error closing stream. %s", oboe::convertToText(result));
        }
        LOGW("Successfully closed streams");
    }
}


/**
 * Warn in logcat if non-low latency stream is created
 * @param stream: newly created stream
 *
 */
void LiveEffectEngine::warnIfNotLowLatency(oboe::AudioStream *stream) {
    if (stream->getPerformanceMode() != oboe::PerformanceMode::LowLatency) {
        LOGW(
            "Stream is NOT low latency."
            "Check your requested format, sample rate and channel count");
    }
}

void LiveEffectEngine::recordAndWrite(std::future<void> exitFuture) {
    /*
     * exitFuture will trigger when we need to stop.
     */

    int numFrames = 128 * 6;
    int64_t timeoutNanos = 1e6 * (numFrames / (mSampleRate / 1000.0));
    void *audioData = new int16_t[numFrames];

    int32_t actualFramesRead = 0;
    while (mCountCallbacksToDrain > 0) {
        // Drain the input.
        int32_t totalFramesRead = 0;
        do {
            oboe::ResultWithValue<int32_t> result = mRecordingStream->read(audioData,
                                                                           numFrames,
                                                                           0 /* timeout */);
            if (!result) {
                // Ignore errors because input stream may not be started yet.
                break;
            }
            actualFramesRead = result.value();
            totalFramesRead += actualFramesRead;
        } while (actualFramesRead > 0);
        // Only counts if we actually got some data.
        if (totalFramesRead > 0) {
            mCountCallbacksToDrain--;
        }
    }
    recordingDrained = true;

    int32_t framesRead = 0;
    // Continually read from the microphone until closeAllStreams() is called.
    while ((exitFuture).wait_for(std::chrono::nanoseconds(0)) == std::future_status::timeout) {
        oboe::ResultWithValue<int32_t> status =
                mRecordingStream->read(audioData, numFrames, timeoutNanos);
        if (!status) {
            LOGE("input stream read error: %s", oboe::convertToText(status.error()));
            break;
        }
        framesRead = status.value();

        if (framesRead < numFrames) {
            LOGE("%s framesRead=%d < numFrames=%d", TAG, framesRead, numFrames);
            int32_t bytesPerFrame = mRecordingStream->getChannelCount() *
                                    mRecordingStream->getBytesPerSample();

            // uint8_t because 8 bits in a byte.
            uint8_t *padPos =
                    static_cast<uint8_t *>(audioData) + framesRead * bytesPerFrame;
            memset(padPos, 0, static_cast<size_t>((numFrames - framesRead) * bytesPerFrame));
        }

        int16_t *newData = static_cast<int16_t *>(audioData);

        for (int i = 0; i < numFrames; i++) {
            queue.push(newData[i]);
        }
        LOGI("%s recordAndWriteThread wrote to queue. Queue Size: %d", TAG, queue.size());
    }
}

std::thread LiveEffectEngine::createRecordAndWriteThread() {
    threadExitSignal = std::promise<void>();
    return std::thread (&LiveEffectEngine::recordAndWrite, this,
                        threadExitSignal.get_future());
}


/**
 * Handles playback stream's audio request. In this sample, we simply block-read
 * from the record stream for the required samples.
 *
 * @param oboeStream: the playback stream that requesting additional samples
 * @param audioData:  the buffer to load audio samples for playback stream
 * @param numFrames:  number of frames to load to audioData buffer
 * @return: DataCallbackResult::Continue.
 */
oboe::DataCallbackResult LiveEffectEngine::onAudioReady(
        oboe::AudioStream *oboeStream, void *audioData, int32_t numFrames) {
    assert(oboeStream == mPlayStream);
    LOGI("%s onAudioReady called with queue_size=%d, numFrames=%d", TAG, queue.size(), numFrames);

    int32_t numBytes = numFrames * oboeStream->getBytesPerFrame();
    memset(audioData, 0 /* value */, numBytes);
    int16_t *filteredData16 = static_cast<int16_t *>(audioData);
    for (int i = 0; i < numFrames; i++) {
        if (!queue.pop(filteredData16[i])) {
            filteredData16[i] = 0;
        }
    }

    return oboe::DataCallbackResult::Continue;
}

/**
 * Oboe notifies the application for "about to close the stream".
 *
 * @param oboeStream: the stream to close
 * @param error: oboe's reason for closing the stream
 */
void LiveEffectEngine::onErrorBeforeClose(oboe::AudioStream *oboeStream,
                                          oboe::Result error) {
    LOGE("%s stream Error before close: %s",
         oboe::convertToText(oboeStream->getDirection()),
         oboe::convertToText(error));
}

/**
 * Oboe notifies application that "the stream is closed"
 *
 * @param oboeStream
 * @param error
 */
void LiveEffectEngine::onErrorAfterClose(oboe::AudioStream *oboeStream,
                                         oboe::Result error) {
    LOGE("%s stream Error after close: %s",
         oboe::convertToText(oboeStream->getDirection()),
         oboe::convertToText(error));
}
