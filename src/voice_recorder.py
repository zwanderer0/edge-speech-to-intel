#!/usr/bin/env python3
"""
Voice Recorder with AI Transcription and Summarization
=====================================================

An intelligent voice recording application that uses Voice Activity Detection (VAD)
to automatically capture speech, transcribe it using Whisper, and summarize content
using a local Large Language Model.

Author: zwanderer
License: MIT
Version: 1.0.0

Features:
- Real-time Voice Activity Detection using WebRTC VAD
- Automatic recording start/stop based on speech detection
- Whisper-based transcription
- Local LLM summarization with configurable models
- Tkinter GUI with real-time status updates
- Configurable recording intervals and sensitivity

Dependencies:
- pyaudio: Audio recording
- webrtcvad: Voice Activity Detection
- numpy: Audio data processing
- tkinter: GUI interface
- llama-cpp-python: Local LLM inference
- whisper.cpp: Audio transcription (external binary)
"""

import pyaudio
import wave
import webrtcvad
import numpy as np
import tkinter as tk
import os
import subprocess
import threading
import time
from datetime import datetime
from llama_cpp import Llama
from tkinter import ttk
from typing import Optional, List


class AudioConfig:
    """Audio recording configuration constants."""
    CHUNK_DURATION_MS = 30          # Duration of a chunk in milliseconds
    RATE = 16000                    # Sample rate in Hz (16kHz required for VAD)
    CHUNK_SIZE = int(RATE * CHUNK_DURATION_MS / 1000)  # Chunk size in samples
    FORMAT = pyaudio.paInt16        # Audio format (16-bit PCM)
    CHANNELS = 1                    # Mono audio
    SILENCE_DURATION = 10           # Duration of silence in seconds to stop recording
    UPDATE_INTERVAL = 10            # Interval to save recording in seconds


class AppConfig:
    """Application configuration constants."""
    RECORDINGS_DIR = "recordings"                    # Directory to save recordings
    DEFAULT_TRANSCRIPTION_INTERVAL = 30             # Default transcription interval in minutes
    WHISPER_MODEL_PATH = "./models/ggml-large-v3.bin"  # Path to Whisper model
    WHISPER_BINARY = "./main"                        # Path to whisper.cpp binary
    
    # NOTE: Replace this path with your local LLM model
    # You can use any GGUF format model (e.g., Mistral, Llama, etc.)
    LLM_MODEL_PATH = "/Users/zwanderer/Desktop/2024/Summ/mistral-7b-instruct-v0.2.Q4_K_M.gguf"
    LLM_CONTEXT_SIZE = 60000                         # LLM context window size


class VoiceRecorderApp:
    """
    Main application class for voice recording with AI transcription and summarization.
    
    This application provides a GUI interface for recording audio with automatic
    speech detection, transcription using Whisper, and summarization using a local LLM.
    """
    
    def __init__(self, master: tk.Tk):
        """
        Initialize the Voice Recorder application.
        
        Args:
            master: The root Tkinter window
        """
        self.master = master
        master.title("AI Voice Recorder - zwanderer")
        
        # Initialize audio components
        self._setup_audio()
        
        # Initialize application state
        self._init_state()
        
        # Create GUI components
        self.create_widgets()
        
        # Start background transcription scheduler
        self._start_transcription_scheduler()
        
        # Handle application closing
        master.protocol("WM_DELETE_WINDOW", self.on_closing)
    
    def _setup_audio(self) -> None:
        """Initialize PyAudio and WebRTC VAD components."""
        # Ensure recordings directory exists
        if not os.path.exists(AppConfig.RECORDINGS_DIR):
            os.makedirs(AppConfig.RECORDINGS_DIR)
        
        # Initialize PyAudio
        self.pyaudio = pyaudio.PyAudio()
        
        # Initialize WebRTC VAD (Voice Activity Detection)
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Moderate sensitivity (0-3, where 3 is most aggressive)
    
    def _init_state(self) -> None:
        """Initialize application state variables."""
        self.is_running = False
        self.frames: List[bytes] = []
        self.silent_frames = 0
        self.speech_duration = 0.0
        self.is_recording = False
        self.chunk_index = 0
        self.output_file = self._get_output_filename()
        self.stream: Optional[pyaudio.Stream] = None
        self.recording_thread: Optional[threading.Thread] = None
        self.transcription_thread: Optional[threading.Thread] = None
        self.start_time = 0.0
    
    def create_widgets(self) -> None:
        """Create and layout GUI widgets."""
        # VAD Sensitivity Control
        sensitivity_label = tk.Label(self.master, text="VAD Sensitivity (0-3):")
        sensitivity_label.grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        
        self.sensitivity = tk.IntVar(value=1)
        self.sensitivity_scale = tk.Scale(
            self.master, from_=0, to=3, orient=tk.HORIZONTAL, 
            variable=self.sensitivity, length=200
        )
        self.sensitivity_scale.grid(row=0, column=1, sticky=tk.W, padx=5, pady=5)
        
        # Transcription Interval Control
        interval_label = tk.Label(self.master, text="Transcription Interval (minutes):")
        interval_label.grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        
        self.minutes = tk.IntVar(value=AppConfig.DEFAULT_TRANSCRIPTION_INTERVAL)
        self.minutes_scale = tk.Scale(
            self.master, from_=1, to=60, orient=tk.HORIZONTAL,
            variable=self.minutes, length=200
        )
        self.minutes_scale.grid(row=1, column=1, sticky=tk.W, padx=5, pady=5)
        
        # Control Buttons
        self.record_button = tk.Button(
            self.master, text="Start Recording", command=self.start_recording,
            bg="green", fg="white", font=("Arial", 12, "bold")
        )
        self.record_button.grid(row=2, column=0, sticky=tk.W, padx=5, pady=10)
        
        self.stop_button = tk.Button(
            self.master, text="Stop Recording", command=self.stop_recording,
            state=tk.DISABLED, bg="red", fg="white", font=("Arial", 12, "bold")
        )
        self.stop_button.grid(row=2, column=1, sticky=tk.W, padx=5, pady=10)
        
        self.transcribe_button = tk.Button(
            self.master, text="Manual Transcribe", command=self.transcribe_audio,
            bg="blue", fg="white", font=("Arial", 10)
        )
        self.transcribe_button.grid(row=3, column=0, sticky=tk.W, padx=5, pady=5)
        
        # Status Display
        self.status_label = tk.Label(
            self.master, text="Status: Idle", fg="blue", font=("Arial", 12, "bold")
        )
        self.status_label.grid(row=4, column=0, columnspan=2, sticky=tk.W, padx=5, pady=5)
        
        # Timer and Progress
        self.timer_label = tk.Label(self.master, text="00:00 / 00:00", font=("Courier", 14))
        self.timer_label.grid(row=5, column=0, columnspan=2, sticky=tk.W, padx=5, pady=5)
        
        self.progress_bar = ttk.Progressbar(self.master, length=400, mode='determinate')
        self.progress_bar.grid(row=6, column=0, columnspan=2, sticky=tk.W+tk.E, padx=5, pady=5)
        
        # Log Display
        log_label = tk.Label(self.master, text="System Log:", font=("Arial", 10, "bold"))
        log_label.grid(row=7, column=0, sticky=tk.W, padx=5, pady=(10, 0))
        
        self.log_text = tk.Text(self.master, height=8, width=60, font=("Courier", 9))
        self.log_text.grid(row=8, column=0, columnspan=2, sticky=tk.W+tk.E, padx=5, pady=5)
        
        # Transcript Display
        transcript_label = tk.Label(self.master, text="Transcripts:", font=("Arial", 10, "bold"))
        transcript_label.grid(row=9, column=0, sticky=tk.W, padx=5, pady=(10, 0))
        
        self.transcript_text = tk.Text(self.master, height=8, width=60, font=("Arial", 9))
        self.transcript_text.grid(row=10, column=0, columnspan=2, sticky=tk.W+tk.E, padx=5, pady=5)
        
        # Summary Display
        summary_label = tk.Label(self.master, text="AI Summaries:", font=("Arial", 10, "bold"))
        summary_label.grid(row=11, column=0, sticky=tk.W, padx=5, pady=(10, 0))
        
        self.summary_text = tk.Text(self.master, height=8, width=60, font=("Arial", 9))
        self.summary_text.grid(row=12, column=0, columnspan=2, sticky=tk.W+tk.E, padx=5, pady=5)
    
    def log(self, message: str) -> None:
        """
        Log a message to the GUI log display and console.
        
        Args:
            message: The message to log
        """
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        
        self.log_text.insert(tk.END, f"{formatted_message}\n")
        self.log_text.see(tk.END)
        print(formatted_message)
    
    def update_summary(self, summary: str, timestamp: str) -> None:
        """
        Update the summary display with new AI-generated summary.
        
        Args:
            summary: The AI-generated summary text
            timestamp: Timestamp when the summary was generated
        """
        summary_header = f"=== Summary at {timestamp} ===\n"
        self.summary_text.insert(tk.END, summary_header + summary + "\n\n")
        self.summary_text.see(tk.END)
    
    def update_transcript(self, transcript: str, timestamp: str) -> None:
        """
        Update the transcript display with new transcription.
        
        Args:
            transcript: The transcribed text
            timestamp: Timestamp when the transcription was generated
        """
        transcript_header = f"=== Transcript at {timestamp} ===\n"
        self.transcript_text.insert(tk.END, transcript_header + transcript + "\n\n")
        self.transcript_text.see(tk.END)
    
    def _get_output_filename(self) -> str:
        """
        Generate a unique filename for the current recording.
        
        Returns:
            Path to the output WAV file
        """
        current_date = datetime.now().strftime("%Y%m%d")
        return os.path.join(
            AppConfig.RECORDINGS_DIR, 
            f"recording_{current_date}_{self.chunk_index:04d}.wav"
        )
    
    def start_recording(self) -> None:
        """Start the voice recording process."""
        self.is_running = True
        self.record_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.NORMAL)
        
        # Configure VAD sensitivity
        self.vad.set_mode(self.sensitivity.get())
        
        # Reset recording state
        self.frames = []
        self.silent_frames = 0
        self.speech_duration = 0
        self.is_recording = False
        self.chunk_index = 0
        self.output_file = self._get_output_filename()
        
        # Start audio stream
        self._open_stream()
        
        self.log("Recording session started - waiting for speech...")
        self.start_time = time.time()
        self._update_timer()
        
        # Start recording thread
        self.recording_thread = threading.Thread(target=self._record_audio, daemon=True)
        self.recording_thread.start()
    
    def stop_recording(self) -> None:
        """Stop the voice recording process."""
        self.is_running = False
        self.record_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        
        self._close_stream()
        self._save_recording(final=True)
        self.log("Recording session stopped")
        self.status_label.config(text="Status: Idle", fg="blue")
    
    def _open_stream(self) -> None:
        """Open the audio input stream."""
        self.stream = self.pyaudio.open(
            format=AudioConfig.FORMAT,
            channels=AudioConfig.CHANNELS,
            rate=AudioConfig.RATE,
            input=True,
            frames_per_buffer=AudioConfig.CHUNK_SIZE
        )
    
    def _close_stream(self) -> None:
        """Close the audio input stream."""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None
    
    def _record_audio(self) -> None:
        """
        Main recording loop that processes audio chunks and detects speech.
        
        This method runs in a separate thread and continuously processes audio
        chunks using WebRTC VAD to detect speech activity.
        """
        last_save_time = time.time()
        
        while self.is_running:
            try:
                # Read audio chunk
                data = self.stream.read(AudioConfig.CHUNK_SIZE, exception_on_overflow=False)
                
                # Use WebRTC VAD to detect speech in the current chunk
                is_speech = self.vad.is_speech(data, AudioConfig.RATE)
                
                if is_speech:
                    # Speech detected - start/continue recording
                    if not self.is_recording:
                        self.log("Speech detected - recording started")
                        self.is_recording = True
                    
                    self.frames.append(data)
                    self.silent_frames = 0
                    self.speech_duration += AudioConfig.CHUNK_DURATION_MS / 1000
                    self.status_label.config(text="Status: Recording Speech", fg="green")
                else:
                    # No speech detected
                    self.silent_frames += 1
                    if self.is_recording:
                        self.status_label.config(text="Status: Paused (silence)", fg="orange")
                    else:
                        self.status_label.config(text="Status: Listening...", fg="blue")
                
                # Periodic save of accumulated audio
                current_time = time.time()
                if current_time - last_save_time >= AudioConfig.UPDATE_INTERVAL:
                    self._save_recording()
                    last_save_time = current_time
                
                # Stop recording after prolonged silence
                silence_threshold = AudioConfig.SILENCE_DURATION * AudioConfig.RATE / AudioConfig.CHUNK_SIZE
                if self.is_recording and self.silent_frames > silence_threshold:
                    self.log(f"Prolonged silence ({AudioConfig.SILENCE_DURATION}s) - saving segment")
                    self._save_recording()
                    self.is_recording = False
                
                # Update GUI
                self.master.update()
                
            except Exception as e:
                self.log(f"Error in recording loop: {e}")
    
    def _save_recording(self, final: bool = False) -> None:
        """
        Save the current audio frames to a WAV file.
        
        Args:
            final: Whether this is the final save (creates new file) or incremental
        """
        if not self.frames:
            return
        
        try:
            mode = 'wb' if final else 'ab'
            with wave.open(self.output_file, mode) as wf:
                wf.setnchannels(AudioConfig.CHANNELS)
                wf.setsampwidth(self.pyaudio.get_sample_size(AudioConfig.FORMAT))
                wf.setframerate(AudioConfig.RATE)
                wf.writeframes(b''.join(self.frames))
            
            self.frames = []
            
            if final:
                self.log(f"Recording saved: {self.output_file}")
                self.chunk_index += 1
                self.output_file = self._get_output_filename()
            
        except Exception as e:
            self.log(f"Error saving recording: {e}")
    
    def _start_transcription_scheduler(self) -> None:
        """Start the background thread that schedules automatic transcriptions."""
        self.transcription_thread = threading.Thread(
            target=self._schedule_transcription_check, daemon=True
        )
        self.transcription_thread.start()
    
    def _schedule_transcription_check(self) -> None:
        """
        Background thread that checks if transcription should be triggered.
        
        Transcription is triggered when the accumulated speech duration
        exceeds the configured interval.
        """
        while True:
            time.sleep(60)  # Check every minute
            
            if self.speech_duration >= self.minutes.get() * 60:
                self.log(f"Auto-transcription triggered ({self.speech_duration:.1f}s of speech)")
                transcription_thread = threading.Thread(target=self.transcribe_audio, daemon=True)
                transcription_thread.start()
                self.speech_duration = 0  # Reset counter
    
    def transcribe_audio(self) -> None:
        """
        Transcribe the latest recording using Whisper and generate an AI summary.
        
        This method calls the external whisper.cpp binary to perform transcription,
        then uses a local LLM to generate a summary of the transcribed content.
        """
        try:
            # Ensure latest recording is saved
            self._save_recording(final=True)
            
            # Determine input file
            input_wav = os.path.join(
                AppConfig.RECORDINGS_DIR,
                f"recording_{datetime.now().strftime('%Y%m%d')}_{self.chunk_index - 1:04d}.wav"
            )
            
            if not os.path.exists(input_wav):
                self.log(f"Warning: Audio file not found: {input_wav}")
                return
            
            output_txt = os.path.splitext(input_wav)[0] + ".txt"
            
            # Build whisper.cpp command
            command = [
                AppConfig.WHISPER_BINARY,
                "-f", input_wav,
                "-m", AppConfig.WHISPER_MODEL_PATH,
                "-tdrz",  # Enable timestamps, diarization, and other features
                "-otxt", output_txt
            ]
            
            self.log(f"Starting transcription: {os.path.basename(input_wav)}")
            self.log(f"Command: {' '.join(command)}")
            
            # Execute transcription
            result = subprocess.run(command, capture_output=True, text=True, timeout=300)
            
            if result.returncode != 0:
                self.log(f"Transcription failed: {result.stderr}")
                return
            
            # Process transcription results
            if os.path.exists(output_txt):
                with open(output_txt, 'r', encoding='utf-8') as file:
                    transcript = file.read()
                
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self.update_transcript(transcript, timestamp)
                self.log(f"Transcription completed: {len(transcript)} characters")
                
                # Generate AI summary
                self._summarize_transcript(output_txt, timestamp)
            else:
                # Fallback: try to find the most recent transcript
                latest_txt = self._find_latest_txt_file()
                if latest_txt:
                    self.log(f"Using most recent transcript: {latest_txt}")
                    with open(latest_txt, 'r', encoding='utf-8') as file:
                        transcript = file.read()
                    
                    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    self.update_transcript(transcript, timestamp)
                    self._summarize_transcript(latest_txt, timestamp)
                else:
                    self.log("No transcription file found")
            
        except subprocess.TimeoutExpired:
            self.log("Transcription timed out (5 minutes)")
        except Exception as e:
            self.log(f"Error during transcription: {e}")
    
    def _find_latest_txt_file(self) -> Optional[str]:
        """
        Find the most recently created transcript file.
        
        Returns:
            Path to the latest transcript file, or None if no files found
        """
        txt_files = [f for f in os.listdir(AppConfig.RECORDINGS_DIR) if f.endswith('.txt')]
        if not txt_files:
            return None
        
        latest_file = max(
            txt_files, 
            key=lambda f: os.path.getctime(os.path.join(AppConfig.RECORDINGS_DIR, f))
        )
        return os.path.join(AppConfig.RECORDINGS_DIR, latest_file)
    
    def _summarize_transcript(self, transcript_file: str, timestamp: str) -> None:
        """
        Generate an AI summary of the transcript using a local LLM.
        
        Args:
            transcript_file: Path to the transcript text file
            timestamp: Timestamp for the summary
        """
        try:
            self.log("Initializing LLM for summarization...")
            
            # Initialize the local LLM
            # NOTE: Replace AppConfig.LLM_MODEL_PATH with your model path
            llm = Llama(
                model_path=AppConfig.LLM_MODEL_PATH,
                n_ctx=AppConfig.LLM_CONTEXT_SIZE,
                verbose=False
            )
            
            # Read transcript content
            with open(transcript_file, 'r', encoding='utf-8') as file:
                text_content = file.read()
            
            if len(text_content.strip()) < 10:
                self.log("Transcript too short for summarization")
                return
            
            # Create prompt for summarization
            # This prompt format works well with Mistral and similar instruction-tuned models
            prompt = f"""[INST] Analyze the following conversation transcript and provide:

1. A concise summary in 2-3 bullet points
2. Key action items or to-dos mentioned (max 3 items)
3. Important topics or themes discussed

Transcript:
{text_content}

Please provide a clear, structured response. [/INST]"""
            
            self.log("Generating AI summary...")
            
            # Generate summary using the local LLM
            output = llm.create_completion(
                prompt=prompt,
                max_tokens=200,
                stop=["[/INST]", "</s>"],
                temperature=0.3,
                top_p=0.9
            )
            
            # Extract and display summary
            summary = output['choices'][0]['text'].strip()
            self.update_summary(summary, timestamp)
            self.log("AI summary generated successfully")
            
        except Exception as e:
            self.log(f"Error generating summary: {e}")
            self.log("Make sure your LLM model path is correct in AppConfig.LLM_MODEL_PATH")
    
    def _update_timer(self) -> None:
        """Update the timer display and progress bar."""
        if self.is_running:
            elapsed_time = time.time() - self.start_time
            minutes, seconds = divmod(elapsed_time, 60)
            elapsed_str = f"{int(minutes):02d}:{int(seconds):02d}"
            
            total_duration = self.minutes.get() * 60
            total_minutes, total_seconds = divmod(total_duration, 60)
            total_str = f"{int(total_minutes):02d}:{int(total_seconds):02d}"
            
            self.timer_label.config(text=f"{elapsed_str} / {total_str}")
            
            # Update progress bar
            if total_duration > 0:
                progress = (elapsed_time / total_duration) * 100
                self.progress_bar['value'] = min(progress, 100)
            
            # Schedule next update
            self.master.after(1000, self._update_timer)
    
    def on_closing(self) -> None:
        """Handle application shutdown."""
        self.log("Shutting down application...")
        
        self.is_running = False
        
        # Wait for threads to complete
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=2)
        
        if self.transcription_thread and self.transcription_thread.is_alive():
            self.transcription_thread.join(timeout=2)
        
        # Close audio stream
        self._close_stream()
        
        # Terminate PyAudio
        if hasattr(self, 'pyaudio'):
            self.pyaudio.terminate()
        
        self.master.destroy()


def main():
    """Main entry point for the application."""
    # Create and configure the main window
    root = tk.Tk()
    root.geometry("700x800")
    root.resizable(True, True)
    root.title("AI Voice Recorder - zwanderer")
    
    # Create and run the application
    app = VoiceRecorderApp(root)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("Application interrupted by user")
        app.on_closing()


if __name__ == "__main__":
    main()