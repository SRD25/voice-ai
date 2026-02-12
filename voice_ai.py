import sounddevice as sd
import queue
import json
import subprocess
import threading
import re
import os
from vosk import Model, KaldiRecognizer

# ================= CONFIG =================
MODEL_PATH = "/home/ritesh/vosk/vosk-model-small-en-us-0.15"
OLLAMA_MODEL = "llama3"

# ================ STT SETUP ================
audio_queue = queue.Queue()
stt_model = Model(MODEL_PATH)
recognizer = KaldiRecognizer(stt_model, 16000)

def audio_callback(indata, frames, time, status):
    audio_queue.put(bytes(indata))

# ================ TEXT CLEANING ================
def clean_text(text):
    # Remove markdown and special formatting characters
    text = re.sub(r'[*_`#>\-]', '', text)
    text = re.sub(r'\s+', ' ', text)
    return text.strip()

# ================ TTS THREAD =================
tts_queue = queue.Queue()

def tts_worker():
    while True:
        text = tts_queue.get()
        if text is None:
            break

        subprocess.run(
            [
                "espeak-ng",
                "-v", "en+m3",
                "-s", "180",   # faster speech
                "-p", "45",    # slightly higher pitch
                "-a", "180",   # stronger volume
                text
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

# Start TTS worker thread
threading.Thread(target=tts_worker, daemon=True).start()

# ================ OLLAMA HANDLER ================
def run_ollama(prompt):

    # Optional: reduce internal parallelism
    os.environ["OLLAMA_NUM_PARALLEL"] = "1"

    process = subprocess.Popen(
        ["ollama", "run", OLLAMA_MODEL, "Give short clear answers. " + prompt],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL
    )

    sentence_buffer = ""

    while True:
        raw = process.stdout.read(1)
        if not raw:
            break

        char = raw.decode("utf-8", errors="ignore")
        print(char, end="", flush=True)

        sentence_buffer += char

        # Condition 1: Proper sentence ending
        if char in ".?!":
            cleaned = clean_text(sentence_buffer)
            if cleaned:
                tts_queue.put(cleaned)
            sentence_buffer = ""

        # Condition 2: Prevent long silence (force chunk)
        elif len(sentence_buffer) > 180:
            cleaned = clean_text(sentence_buffer)
            if cleaned:
                tts_queue.put(cleaned)
            sentence_buffer = ""

    # Speak remaining text
    if sentence_buffer.strip():
        cleaned = clean_text(sentence_buffer)
        if cleaned:
            tts_queue.put(cleaned)

# ================ MAIN LOOP =================
print("🎙️ Voice AI ready. Speak...")

with sd.RawInputStream(
    samplerate=16000,
    blocksize=8000,
    dtype='int16',
    channels=1,
    callback=audio_callback
):
    while True:
        data = audio_queue.get()

        if recognizer.AcceptWaveform(data):
            result = json.loads(recognizer.Result())
            text = result.get("text", "")

            if not text:
                continue

            print("\nYou said:", text)

            if "stop listening" in text:
                tts_queue.put("Stopping voice assistant.")
                break

            # Immediate feedback
            tts_queue.put("Processing your request.")

            run_ollama(text)
