//
// Created by daran on 1/12/2017 to be used in ECE420 Sp17 for the first time.
// Modified by dwang49 on 1/1/2018 to adapt to Android 7.0 and Shield Tablet updates.
//
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <string>
#include <jni.h>
#include "ece420_main.h"
#include "ece420_lib.h"
#include "kiss_fft/kiss_fft.h"

#define PI 3.14159265
std::random_device rd;
std::mt19937 gen(rd());

// values near the mean are the most likely
// standard deviation affects the dispersion of generated values from the mean
std::normal_distribution<float> d(200.0, 100.0);


bool robotVoiceEnabled = false;
bool originalVoiceEnabled = false;
bool radioVoiceEnabled = false;
bool vaderVoiceEnabled = false;

extern "C" {
    JNIEXPORT void JNICALL
    Java_com_ece420_newapp_MainActivity_enableRobotVoice(JNIEnv *env, jclass, jboolean);
    JNIEXPORT void JNICALL
    Java_com_ece420_newapp_MainActivity_enableRadioVoice(JNIEnv *env, jclass, jboolean);
    JNIEXPORT void JNICALL
    Java_com_ece420_newapp_MainActivity_enableVaderVoice(JNIEnv *env, jclass, jboolean);
    JNIEXPORT void JNICALL
    Java_com_ece420_newapp_MainActivity_enableOriginalVoice(JNIEnv *env, jclass, jboolean);
}

// Student Variables
#define EPOCH_PEAK_REGION_WIGGLE 30
#define VOICED_THRESHOLD 200000000
#define FRAME_SIZE 1024
#define BUFFER_SIZE (3 * FRAME_SIZE)
#define DELAY_BUFFER_SIZE 22050
#define MAX_REVERB_BUFFER_SIZE 22050
#define WINDOW_SIZE 1024
#define HOP_SIZE 512
#define F_S 48000
float bufferIn[BUFFER_SIZE] = {};
float bufferOut[BUFFER_SIZE] = {};
//float reverbBuffer[MAX_REVERB_BUFFER_SIZE] = {};

void ece420ProcessFrame(sample_buf *dataBuf) {
    // Keep in mind, we only have 20ms to process each buffer!
//    LOGD("RobotVoiceEnabled: %d", robotVoiceEnabled);
    if(originalVoiceEnabled){
        LOGD("Original Selected");
    }
    else{
        struct timeval start;
        struct timeval end;
        gettimeofday(&start, NULL);

        // Data is encoded in signed PCM-16, little-endian, mono
        int16_t data[FRAME_SIZE];
        for (int i = 0; i < FRAME_SIZE; i++) {
            data[i] = ((uint16_t) dataBuf->buf_[2 * i]) | (((uint16_t) dataBuf->buf_[2 * i + 1]) << 8);
        }
        LOGD("DATA BUF IS READ");

        // Buffer: Past | Present | Future
        // Shift our old data back to make room for the new data
        for (int i = 0; i < 2 * FRAME_SIZE; i++) { // moves the Future and Present back
            bufferIn[i] = bufferIn[i + FRAME_SIZE - 1];
        }
        LOGD("BUFFER IN SHIFTED BACK!");

        // Finally, put in our new data. // incoming data is now the future
        for (int i = 0; i < FRAME_SIZE; i++) {
            bufferIn[i + 2 * FRAME_SIZE - 1] = (float) data[i];
        }
        LOGD("BUFFER IN POPULATED WITH DATA BUF!");

        if (robotVoiceEnabled) {
            LOGD("ROBOT VOICE IS ENABLED!");
            // Perform FFT on the input buffer
//        kiss_fft_cpx prev[FRAME_SIZE], prev_fft[FRAME_SIZE];
            kiss_fft_cpx curr[FRAME_SIZE], curr_fft[FRAME_SIZE];
            kiss_fft_cpx curr2[FRAME_SIZE], curr_fft2[FRAME_SIZE];
            kiss_fft_cpx curr3[FRAME_SIZE], curr_fft3[FRAME_SIZE];


            // grab the present section with overlap in both past and future.
            for (int i = 0; i < FRAME_SIZE; i++) {
                float windowCoeff = getHanningCoef(FRAME_SIZE, i);
                curr[i].r = bufferIn[i+FRAME_SIZE-512]* windowCoeff;
                curr[i].i = 0;

                curr2[i].r = bufferIn[i+FRAME_SIZE] * windowCoeff;
                curr2[i].i = 0;

                curr3[i].r = bufferIn[i+FRAME_SIZE+512] * windowCoeff;
                curr3[i].i = 0;
            }
            kiss_fft_cfg cfg = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);
            kiss_fft_cfg cfg2 = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);
            kiss_fft_cfg cfg3 = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);

            kiss_fft(cfg, curr, curr_fft);
            kiss_fft(cfg2, curr2, curr_fft2);
            kiss_fft(cfg3, curr3, curr_fft3);

//        LOGD("first value: %f", curr[0].r);
//        LOGD("%f", curr[1].r);
//        LOGD("%f", curr[2].r);
//        LOGD("%f", curr[3].r);
//        LOGD("%f", curr[4].r);


            free(cfg);
            free(cfg2);
            free(cfg3);

            // Flatten phase to 0 by setting the imaginary part to 0 and keeping the magnitude as the real part
            for (int i = 0; i < FRAME_SIZE; i++) {
                float magnitude1 = sqrt(curr_fft[i].r * curr_fft[i].r + curr_fft[i].i * curr_fft[i].i);
                curr_fft[i].r = magnitude1;
                curr_fft[i].i = 0;

                float magnitude2 = sqrt(curr_fft2[i].r * curr_fft2[i].r + curr_fft2[i].i * curr_fft2[i].i);
                curr_fft2[i].r = magnitude2;
                curr_fft2[i].i = 0;

                float magnitude3 = sqrt(curr_fft3[i].r * curr_fft3[i].r + curr_fft3[i].i * curr_fft3[i].i);
                curr_fft3[i].r = magnitude3;
                curr_fft3[i].i = 0;
            }

            // Perform inverse FFT
            cfg = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);
            cfg2 = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);
            cfg3 = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);


            kiss_fft(cfg, curr_fft, curr);
            kiss_fft(cfg2, curr_fft2, curr2);
            kiss_fft(cfg3, curr_fft3, curr3);

            free(cfg);
            free(cfg2);
            free(cfg3);

//        LOGD("first value, second pass : %f", curr[0].r/1024);
//        LOGD("%f", curr[1].r/1024);
//        LOGD("%f", curr[2].r/1024);
//        LOGD("%f", curr[3].r/1024);
//        LOGD("%f", curr[4].r/1024);

            // Float array for the IFFT
            float processedFrame1[FRAME_SIZE];
            float processedFrame2[FRAME_SIZE];
            float processedFrame3[FRAME_SIZE];

            for (int i = 0; i < FRAME_SIZE; i++) {
                processedFrame1[i] = curr[i].r/FRAME_SIZE;
                processedFrame2[i] = curr2[i].r/FRAME_SIZE;
                processedFrame3[i] = curr3[i].r/FRAME_SIZE;
            }

//         Overlap and add based on the synthesis size.
            overlapAddArray(bufferOut, processedFrame1, 512+1024, FRAME_SIZE); // delay by 20 ms for some delayed playback.
            overlapAddArray(bufferOut, processedFrame2, FRAME_SIZE+1024, FRAME_SIZE);
            overlapAddArray(bufferOut, processedFrame3, FRAME_SIZE+512+1024, FRAME_SIZE);
        }
        else if(radioVoiceEnabled){
            LOGD("Radio Selected");
            // Perform FFT on the input buffer
            kiss_fft_cpx curr[FRAME_SIZE], curr_fft[FRAME_SIZE];
            kiss_fft_cpx curr2[FRAME_SIZE], curr_fft2[FRAME_SIZE];
            kiss_fft_cpx curr3[FRAME_SIZE], curr_fft3[FRAME_SIZE];

            for (int i = 0; i < FRAME_SIZE; i++) {
                float windowCoeff = getHanningCoef(FRAME_SIZE, i);
                curr[i].r = bufferIn[i+FRAME_SIZE-512]* windowCoeff;
                curr[i].i = 0;

                curr2[i].r = bufferIn[i+FRAME_SIZE] * windowCoeff;
                curr2[i].i = 0;

                curr3[i].r = bufferIn[i+FRAME_SIZE+512] * windowCoeff;
                curr3[i].i = 0;
            }
            kiss_fft_cfg cfg = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);
            kiss_fft_cfg cfg2 = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);
            kiss_fft_cfg cfg3 = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);
            kiss_fft(cfg, curr, curr_fft);
            kiss_fft(cfg2, curr2, curr_fft2);
            kiss_fft(cfg3, curr3, curr_fft3);
//        LOGD("first value: %f", curr[0].r);
//        LOGD("%f", curr[1].r);
//        LOGD("%f", curr[2].r);
//        LOGD("%f", curr[3].r);
//        LOGD("%f", curr[4].r);


            free(cfg);
            free(cfg2);
            free(cfg3);

            // TODO: Make the bandwidth really really small by setting zeros bins higher and lower than a certain index
            for (int i = 0; i < FRAME_SIZE; i++) {
                if(i > 100 || i < 10) {
                    curr_fft[i].r /= 1000.0;
                    curr_fft[i].i /= 1000.0;
                    curr_fft2[i].r /= 1000.0;
                    curr_fft2[i].i /= 1000.0;
                    curr_fft3[i].r /= 1000.0;
                    curr_fft3[i].i /= 1000.0;
                }else{
                    curr_fft[i].r *= 2;
                    curr_fft[i].i *= 2;
                    curr_fft2[i].r *= 2;
                    curr_fft2[i].i *= 2;
                    curr_fft3[i].r *= 2;
                    curr_fft3[i].i *= 2;
                }
            }

            // Perform inverse FFT
            cfg = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);
            cfg2 = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);
            cfg3 = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);

            kiss_fft(cfg, curr_fft, curr);
            kiss_fft(cfg2, curr_fft2, curr2);
            kiss_fft(cfg3, curr_fft3, curr3);

            free(cfg);
            free(cfg2);
            free(cfg3);
//        LOGD("first value, second pass : %f", curr[0].r/1024);
//        LOGD("%f", curr[1].r/1024);
//        LOGD("%f", curr[2].r/1024);
//        LOGD("%f", curr[3].r/1024);
//        LOGD("%f", curr[4].r/1024);
            float processedFrame1[FRAME_SIZE];
            float processedFrame2[FRAME_SIZE];
            float processedFrame3[FRAME_SIZE];

            for (int i = 0; i < FRAME_SIZE; i++) {
                processedFrame1[i] = curr[i].r/FRAME_SIZE + d(gen);
                processedFrame2[i] = curr2[i].r/FRAME_SIZE + d(gen);
                processedFrame3[i] = curr3[i].r/FRAME_SIZE + d(gen);
            }
            // Overlap and add based on the synthesis size.
            overlapAddArray(bufferOut, processedFrame1, 512+150, FRAME_SIZE);
            overlapAddArray(bufferOut, processedFrame2, FRAME_SIZE+150, FRAME_SIZE);
            overlapAddArray(bufferOut, processedFrame3, FRAME_SIZE+512+150, FRAME_SIZE);

        }
        else if(vaderVoiceEnabled){
            // new Darth Vader effect

            kiss_fft_cpx curr[FRAME_SIZE], curr_fft[FRAME_SIZE];
            kiss_fft_cpx curr2[FRAME_SIZE], curr_fft2[FRAME_SIZE];
//            kiss_fft_cpx curr3[FRAME_SIZE], curr_fft3[FRAME_SIZE];

            for (int i = 0; i < FRAME_SIZE; i++) {
                float windowCoeff = getHanningCoef(FRAME_SIZE, i);
                curr[i].r = bufferIn[i+FRAME_SIZE-512]* windowCoeff;
                curr[i].i = 0;

                curr2[i].r = bufferIn[i+FRAME_SIZE] * windowCoeff;
                curr2[i].i = 0;

//                curr3[i].r = bufferIn[i+FRAME_SIZE+512] * windowCoeff;
//                curr3[i].i = 0;
            }

            kiss_fft_cfg cfg = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);
            kiss_fft_cfg cfg2 = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);
//            kiss_fft_cfg cfg3 = kiss_fft_alloc(FRAME_SIZE, false, 0, 0);

            kiss_fft(cfg, curr, curr_fft);
            kiss_fft(cfg2, curr2, curr_fft2);
//            kiss_fft(cfg3, curr3, curr_fft3);

            float magnitude[1024];
            float prevPhase[1024];
            float phase[1024];

            // need to figure out way to shift the magnitude of each bin to the left.
            for (int i = 0; i < FRAME_SIZE; i++) {
                // Calculate magnitude and phase
                prevPhase[i] = atan(curr_fft[i].i / curr_fft[i].r);
                phase[i] = atan2(curr_fft2[i].i, curr_fft2[i].r);
                magnitude[i] = sqrt(curr_fft2[i].r * curr_fft2[i].r + curr_fft2[i].i * curr_fft2[i].i);

                // Phase difference
                float deltaPhi = phaseDifference(prevPhase[i], phase[i]);
                prevPhase[i] = phase[i]; // Update previous phase

                // Phase adjustment for pitch shift
                float trueFreq = (2 * M_PI * i / FRAME_SIZE) + deltaPhi;
                phase[i] += trueFreq * (0.90 - 1);
            }

            for (int i = 0; i < FRAME_SIZE; i++) {
                // Shift the magnitude bins to the left
                if (i > 3) {
                    magnitude[i - 10] = magnitude[i];
                }
            }

            for (int i = 0; i < FRAME_SIZE; i++) {
                // Convert back to rectangular form
                curr_fft[i].r = magnitude[i] * cos(phase[i]);
                curr_fft[i].i = magnitude[i] * sin(phase[i]);
            }

            // Perform inverse FFT
            cfg = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);
            cfg2 = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);
//            cfg3 = kiss_fft_alloc(FRAME_SIZE, true, 0, 0);

            kiss_fft(cfg, curr_fft, curr);
            kiss_fft(cfg2, curr_fft2, curr2);
//            kiss_fft(cfg3, curr_fft3, curr3);

            free(cfg);
            free(cfg2);
//            free(cfg3);



            // reverb
            float roomSize = 0.5f;
            float damping = 0.6f;
            for (int i = 0; i < 1024; i++) {
                curr[i].r = applyReverb(curr[i].r, roomSize, damping);
                curr2[i].r = applyReverb(curr2[i].r, roomSize, damping);
//                curr3[i].r = applyReverb(curr3[i].r/1024, roomSize, damping);
            }

            float processedFrame1[FRAME_SIZE];
            float processedFrame2[FRAME_SIZE];
//            float processedFrame3[FRAME_SIZE];

            for (int i = 0; i < FRAME_SIZE; i++) {
                processedFrame1[i] = curr[i].r/1024;
                processedFrame2[i] = curr2[i].r/1024;
//                processedFrame3[i] = curr3[i].r/1024;
            }

            float duplicateTrack[FRAME_SIZE];
            float duplicateTrack2[FRAME_SIZE];

            // copy over the buffer
            for (int i = 0; i < FRAME_SIZE; i++) {
                duplicateTrack[i] = processedFrame1[i];
                duplicateTrack2[i] = processedFrame2[i];
            }

            const float cutoffFreq = 3000.0f; // Low-pass filter cutoff frequency in Hz
            float lpFilteredTrack[FRAME_SIZE];
            float lpFilteredTrack2[FRAME_SIZE];
            float rc = 1.0f / (cutoffFreq * 2 * M_PI);
            float dt = 1.0f / F_S;
            float alpha = dt / (rc + dt);

            lpFilteredTrack[0] = duplicateTrack[0];
            lpFilteredTrack2[0] = duplicateTrack2[0];
            for (int i = 1; i < FRAME_SIZE; i++) {
                lpFilteredTrack[i] = lpFilteredTrack[i - 1] + alpha * (duplicateTrack[i] - lpFilteredTrack[i - 1]);
                lpFilteredTrack2[i] = lpFilteredTrack2[i - 1] + alpha * (duplicateTrack2[i] - lpFilteredTrack2[i - 1]);
            }

            float mixRatio = 0.2; // how much we mix the wet and dry tracks.

            for (int i = 0; i < FRAME_SIZE; i++) {
                processedFrame1[i] = mixRatio*processedFrame1[i] + (1-mixRatio) * lpFilteredTrack[i];
                processedFrame2[i] = mixRatio*processedFrame2[i] + (1-mixRatio) * lpFilteredTrack2[i];
            }

            overlapAddArray(bufferOut, processedFrame1, 512+1200, FRAME_SIZE);
            overlapAddArray(bufferOut, processedFrame2, FRAME_SIZE+1200, FRAME_SIZE);
        }
        else{
            LOGD("Nothing Selected");
        }
        for (int i = 0; i < FRAME_SIZE; i++) {
            int16_t newVal = (int16_t) bufferOut[i];
            uint8_t lowByte = (uint8_t) (0x00ff & newVal);
            uint8_t highByte = (uint8_t) ((0xff00 & newVal) >> 8);
            dataBuf->buf_[i * 2] = lowByte;
            dataBuf->buf_[i * 2 + 1] = highByte;
        }

        LOGD("UPDATING CIRCULAR BUFFER FOR OUTPUT!");

        for (int i = 0; i < 2 * FRAME_SIZE; i++) {
            bufferOut[i] = bufferOut[i + FRAME_SIZE - 1];
        }

        for (int i = 0; i < FRAME_SIZE; i++) {
            bufferOut[i + 2 * FRAME_SIZE - 1] = 0;
        }

        gettimeofday(&end, NULL);
        LOGD("Time delay: %ld us",  ((end.tv_sec * 1000000 + end.tv_usec) - (start.tv_sec * 1000000 + start.tv_usec)));
    }
}

void overlapAddArray(float *dest, float *src, int startIdx, int len) {
    int idxLow = startIdx;
    int idxHigh = startIdx + len;

    int padLow = 0;
    int padHigh = 0;
    if (idxLow < 0) {
        padLow = -idxLow;
    }
    if (idxHigh > BUFFER_SIZE) {
        padHigh = BUFFER_SIZE - idxHigh;
    }

    // Finally, reconstruct the buffer
    for (int i = padLow; i < len + padHigh; i++) {
        dest[startIdx + i] += src[i];
    }
}

float applyReverb(float inputSample, float roomSize, float damping) {
    // Reverb settings
    const int maxDelay = 500; // in samples
    int delay = maxDelay * roomSize;
//    LOGD("delay: %d", delay);

    // Feedback and damping
    static int writeIndex = 0;
    int readIndex = (writeIndex + MAX_REVERB_BUFFER_SIZE - delay) % MAX_REVERB_BUFFER_SIZE;
//    LOGD("writeIndex: %d", writeIndex);
//    LOGD("readIndex: %d", readIndex);

    // reverb buffer
    static float reverbBuffer[MAX_REVERB_BUFFER_SIZE] = {};

    // Damping calculation (simple low-pass filter)
    if (!isnan(inputSample)) {
        reverbBuffer[writeIndex] = (reverbBuffer[readIndex] * damping) + (inputSample * (1 - damping));
    }
//    LOGD("reverbBuffer Past value: %f", reverbBuffer[writeIndex]);
    // Mix dry and wet signals
    float output = (inputSample + reverbBuffer[readIndex]) / 2.0f;
//    LOGD("REVERBED VALUE: %f", output);

    // Increment and wrap write index
    writeIndex = (writeIndex + 1) % MAX_REVERB_BUFFER_SIZE;
//    LOGD("new writeIndex: %d", writeIndex);

    return output;
}

float phaseDifference(float phase1, float phase2) {
    float diff = phase2 - phase1;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}



JNIEXPORT void JNICALL
Java_com_ece420_newapp_MainActivity_enableRobotVoice(JNIEnv *env, jclass, jboolean enable) {
    robotVoiceEnabled = enable;
    return;
}

JNIEXPORT void JNICALL
Java_com_ece420_newapp_MainActivity_enableRadioVoice(JNIEnv *env, jclass, jboolean enable) {
    radioVoiceEnabled = enable;
    return;
}

JNIEXPORT void JNICALL
Java_com_ece420_newapp_MainActivity_enableVaderVoice(JNIEnv *env, jclass, jboolean enable) {
    vaderVoiceEnabled = enable;
    return;
}

JNIEXPORT void JNICALL
Java_com_ece420_newapp_MainActivity_enableOriginalVoice(JNIEnv *env, jclass, jboolean enable) {
    originalVoiceEnabled = enable;
    return;
}