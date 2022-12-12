from gtts import gTTS
from io import BytesIO
from pygame import mixer
import time

def speak(s,t):
    # mp3_fp = BytesIO()
    sound = BytesIO()
    tts = gTTS(s, lang='en')
    # tts.write_to_fp(mp3_fp)
    # return mp3_fp

    tts.write_to_fp(sound)
    sound.seek(0)
    mixer.music.load(sound, "mp3")
    mixer.music.play()
    time.sleep(t)
    return

mixer.init()
[speak(str(i), 1) for i in range(1,6)]