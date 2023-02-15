# Voice Activity Detection (VAD) Guide

This document explains the Voice Activity Detection implementation in the AI Voice Recorder system.

**Author:** zwanderer  
**Focus:** WebRTC VAD integration and optimization

## 🎯 What is Voice Activity Detection?

Voice Activity Detection (VAD) is a technology that automatically distinguishes between speech and non-speech audio segments in real-time. It's crucial for:

- **Efficient Recording:** Only captures audio when speech is present
- **Battery Conservation:** Reduces processing during silent periods  
- **Bandwidth Optimization:** Minimizes data transmission
- **Automatic Segmentation:** Creates natural speech boundaries

## 🔧 Technical Implementation

### WebRTC VAD Integration

The system uses Google's WebRTC VAD library, which provides:

```python
import webrtcvad

# Initialize VAD with sensitivity level
vad = webrtcvad.Vad()
vad.set_mode(1)  # Sensitivity: 0 (least) to 3 (most aggressive)

# Process audio chunk (30ms at 16kHz)
is_speech = vad.is_speech(audio_data, sample_rate)
```

### Audio Requirements

WebRTC VAD has specific requirements:

| Parameter | Requirement | Reason |
|-----------|------------|---------|
| Sample Rate | 8kHz, 16kHz, 32kHz, 48kHz | VAD algorithm optimization |
| Bit Depth | 16-bit PCM | Expected input format |
| Channels | Mono | VAD processes single channel |
| Chunk Size | 10ms, 20ms, 30ms | Frame-based analysis |

**Our Configuration:**
```python
RATE = 16000                    # 16kHz for voice quality
CHUNK_DURATION_MS = 30          # 30ms chunks for balance
CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)  # 480 samples
```

## 📊 Sensitivity Levels

### Level 0 - Least Aggressive
- **Use Case:** Quiet environments, close microphone
- **Characteristics:** May miss quiet speech
- **False Positives:** Very few
- **False Negatives:** Higher chance

### Level 1 - Moderate (Recommended)
- **Use Case:** Normal indoor environments
- **Characteristics:** Balanced detection
- **False Positives:** Low
- **False Negatives:** Low

### Level 2 - More Aggressive  
- **Use Case:** Noisy environments, distant microphone
- **Characteristics:** Catches more speech
- **False Positives:** Some background noise
- **False Negatives:** Reduced

### Level 3 - Most Aggressive
- **Use Case:** Very noisy environments
- **Characteristics:** Detects any potential speech
- **False Positives:** Higher (music, TV, etc.)
- **False Negatives:** Minimal

## 🎛 Configuration in Application

### Main Application (Python)

```python
class AudioConfig:
    CHUNK_DURATION_MS = 30      # VAD analysis window
    RATE = 16000               # Sample rate (required for VAD)
    SILENCE_DURATION = 10      # Seconds before stopping recording

class VoiceRecorderApp:
    def __init__(self):
        # Initialize WebRTC VAD
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Default sensitivity
    
    def _record_audio(self):
        while self.is_running:
            # Read audio chunk
            data = self.stream.read(CHUNK_SIZE)
            
            # VAD analysis
            is_speech = self.vad.is_speech(data, RATE)
            
            if is_speech:
                # Start recording speech
                self.frames.append(data)
                self.silent_frames = 0
            else:
                # Count silent frames
                self.silent_frames += 1
```

### ESP32 Implementation

```cpp
// Simple energy-based VAD for ESP32
bool performVAD(int16_t* buffer, size_t samples) {
    long sumSquares = 0;
    int16_t maxAmplitude = 0;
    int zeroCrossings = 0;
    
    for (size_t i = 0; i < samples; i++) {
        sumSquares += (long)buffer[i] * buffer[i];
        
        int16_t absValue = abs(buffer[i]);
        if (absValue > maxAmplitude) {
            maxAmplitude = absValue;
        }
        
        if (i > 0 && ((buffer[i-1] >= 0) != (buffer[i] >= 0))) {
            zeroCrossings++;
        }
    }
    
    int16_t rmsLevel = sqrt(sumSquares / samples);
    
    // Voice activity conditions
    return (rmsLevel > VAD_THRESHOLD) && 
           (maxAmplitude > MIN_AUDIO_LEVEL) && 
           (zeroCrossings > 5) && 
           (zeroCrossings < samples / 4);
}
```

## ⚡ Real-Time Processing Flow

### 1. Audio Capture
```
Microphone → ADC → I2S Buffer → 30ms Chunks
```

### 2. VAD Analysis
```
Audio Chunk → WebRTC VAD → Speech/Non-Speech Decision
```

### 3. Recording Logic
```python
if is_speech:
    if not recording:
        start_recording()
    add_to_buffer(audio_chunk)
    reset_silence_counter()
else:
    increment_silence_counter()
    if silence_counter > threshold:
        stop_recording()
        save_audio_file()
```

### 4. State Management
```
Idle → Speech Detected → Recording → Silence → Pause → Speech → Resume
                      ↓
                   Save File
```

## 🔍 Performance Optimization

### Reducing False Positives

1. **Environment-Specific Tuning**
   ```python
   # Quiet office
   vad.set_mode(0)
   
   # Open office
   vad.set_mode(1)
   
   # Noisy environment  
   vad.set_mode(2)
   ```

2. **Minimum Speech Duration**
   ```python
   MIN_SPEECH_DURATION = 0.5  # seconds
   
   if speech_duration < MIN_SPEECH_DURATION:
       # Ignore short bursts
       continue
   ```

3. **Post-Processing Filters**
   ```python
   def smooth_vad_decisions(decisions, window_size=5):
       # Apply majority voting in sliding window
       smoothed = []
       for i in range(len(decisions)):
           window = decisions[max(0, i-window_size):i+window_size+1]
           smoothed.append(sum(window) > len(window) // 2)
       return smoothed
   ```

### Reducing False Negatives

1. **Lower Sensitivity Threshold**
   ```python
   # More sensitive detection
   vad.set_mode(2)
   ```

2. **Shorter Silence Threshold**
   ```python
   SILENCE_DURATION = 5  # Reduce from 10 seconds
   ```

3. **Context-Aware Detection**
   ```python
   # Continue recording during brief pauses in continuous speech
   if recent_speech_activity and silence_duration < 2:
       continue_recording = True
   ```

## 📈 Performance Metrics

### Evaluation Criteria

1. **Accuracy:** Correct speech/non-speech classification
2. **Latency:** Time from speech start to detection
3. **Robustness:** Performance in various environments
4. **Efficiency:** CPU and memory usage

### Typical Performance

| Environment | Accuracy | Latency | False Pos. | False Neg. |
|-------------|----------|---------|------------|------------|
| Quiet Office | 95%+ | ~60ms | <1% | ~3% |
| Home | 90%+ | ~90ms | ~3% | ~5% |
| Cafe/Public | 85%+ | ~120ms | ~8% | ~7% |
| Noisy Street | 75%+ | ~150ms | ~15% | ~10% |

### Measurement Code

```python
def measure_vad_performance(audio_file, ground_truth):
    # Load audio and annotations
    audio, sr = librosa.load(audio_file, sr=16000)
    
    # Process with VAD
    vad_results = []
    for i in range(0, len(audio), CHUNK_SIZE):
        chunk = audio[i:i+CHUNK_SIZE]
        if len(chunk) == CHUNK_SIZE:
            chunk_bytes = (chunk * 32767).astype(np.int16).tobytes()
            vad_results.append(vad.is_speech(chunk_bytes, 16000))
    
    # Compare with ground truth
    accuracy = calculate_accuracy(vad_results, ground_truth)
    return accuracy
```

## 🔧 Troubleshooting Common Issues

### Issue: VAD Not Detecting Quiet Speech

**Symptoms:**
- Soft-spoken users not detected
- Recordings cut off during quiet parts

**Solutions:**
```python
# Lower sensitivity threshold
vad.set_mode(0)  # Less aggressive

# Reduce silence threshold
SILENCE_DURATION = 5  # seconds

# Audio gain adjustment
def amplify_audio(audio_data, gain=2.0):
    return np.clip(audio_data * gain, -32768, 32767)
```

### Issue: Too Many False Positives

**Symptoms:**
- Background noise triggers recording
- Music/TV detected as speech
- Frequent unwanted recordings

**Solutions:**
```python
# Increase sensitivity threshold
vad.set_mode(3)  # More aggressive filtering

# Add frequency analysis
def is_speech_frequency(audio_chunk):
    # Speech typically 85-255 Hz fundamental
    fft = np.fft.fft(audio_chunk)
    freqs = np.fft.fftfreq(len(audio_chunk), 1/16000)
    
    # Check for speech-like frequency content
    speech_band = np.where((freqs >= 85) & (freqs <= 255))
    speech_energy = np.sum(np.abs(fft[speech_band]))
    total_energy = np.sum(np.abs(fft))
    
    return speech_energy / total_energy > 0.1
```

### Issue: High CPU Usage

**Symptoms:**
- System lag during recording
- High processor utilization
- Battery drain on mobile devices

**Solutions:**
```python
# Reduce processing frequency
CHUNK_DURATION_MS = 50  # Increase from 30ms

# Optimize buffer management
def efficient_vad_processing():
    # Process every nth frame instead of all frames
    frame_skip = 2
    if frame_counter % frame_skip == 0:
        is_speech = vad.is_speech(data, RATE)
    else:
        # Use previous result
        is_speech = previous_vad_result
```

## 🧪 Testing and Validation

### Test Suite

```python
def test_vad_configuration():
    """Test VAD with different sensitivity levels"""
    test_files = [
        "quiet_speech.wav",
        "normal_speech.wav", 
        "noisy_speech.wav",
        "silence.wav",
        "music.wav"
    ]
    
    for sensitivity in range(4):
        vad.set_mode(sensitivity)
        for test_file in test_files:
            result = process_test_file(test_file)
            print(f"Mode {sensitivity}, {test_file}: {result}")

def benchmark_performance():
    """Measure VAD processing speed"""
    import time
    
    audio_data = generate_test_audio(duration=10)  # 10 seconds
    chunks = create_chunks(audio_data, CHUNK_SIZE)
    
    start_time = time.time()
    for chunk in chunks:
        vad.is_speech(chunk, RATE)
    end_time = time.time()
    
    processing_time = end_time - start_time
    real_time_factor = 10.0 / processing_time
    
    print(f"Real-time factor: {real_time_factor:.2f}x")
```

### Real-World Testing

1. **Environment Testing**
   - Record in different acoustic environments
   - Test with various background noise levels
   - Validate with multiple speakers

2. **Edge Case Testing**
   - Whispered speech
   - Shouted speech
   - Multiple speakers
   - Music with vocals
   - TV/radio background

3. **Performance Testing**
   - CPU usage monitoring
   - Memory consumption
   - Battery life impact
   - Network bandwidth usage

## 🔮 Advanced VAD Techniques

### Machine Learning Approaches

For future enhancements, consider ML-based VAD:

```python
import tensorflow as tf

class MLBasedVAD:
    def __init__(self, model_path):
        self.model = tf.lite.Interpreter(model_path)
        self.model.allocate_tensors()
    
    def predict(self, audio_features):
        # Extract features (MFCC, spectral features)
        features = self.extract_features(audio_features)
        
        # Run inference
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], features)
        self.model.invoke()
        
        output = self.model.get_tensor(output_details[0]['index'])
        return output[0] > 0.5  # Threshold for speech/non-speech
```

### Hybrid Approaches

Combine WebRTC VAD with additional features:

```python
def hybrid_vad(audio_chunk):
    # Primary VAD decision
    webrtc_decision = vad.is_speech(audio_chunk, RATE)
    
    # Additional checks
    energy_check = check_energy_level(audio_chunk)
    spectral_check = check_spectral_features(audio_chunk)
    temporal_check = check_temporal_consistency()
    
    # Weighted decision
    confidence_score = (
        0.6 * webrtc_decision +
        0.2 * energy_check +
        0.15 * spectral_check +
        0.05 * temporal_check
    )
    
    return confidence_score > 0.5
```

## 📚 Further Reading

- [WebRTC VAD Documentation](https://webrtcvad.readthedocs.io/)
- [Voice Activity Detection Survey](https://arxiv.org/abs/2101.09688)
- [Audio Signal Processing for VAD](https://www.coursera.org/learn/audio-signal-processing)

---

**Author:** zwanderer  
**Last Updated:** 2024  
**License:** MIT