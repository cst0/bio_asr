#!/usr/bin/env python3

from speechbrain.pretrained import VAD
from pydub import AudioSegment
from pydub.utils import mediainfo
import rospkg
import os

rp = rospkg.RosPack()
path = rp.get_path("bio_asr")
data_dir = os.path.join(path, "data")
model_dir = os.path.join(path, "pretrained_models")

# transcription clutters symlinks everywhere if we aren't in this dir
os.chdir(data_dir)
filename = 'afterapplepicking'
audio_file = os.path.join(data_dir, filename+'.mp3')

sound = AudioSegment.from_file(audio_file)
assert type(sound) is AudioSegment
sound = sound.set_frame_rate(16000)
audio_file = os.path.join(data_dir, filename+'_16000.wav')
sound = sound.set_channels(1)
sound.export(audio_file, format='wav')
print(mediainfo(audio_file))

VAD = VAD.from_hparams(
    source="speechbrain/vad-crdnn-libriparty",
    savedir=os.path.join(model_dir,"vad-crdnn-libriparty")
)

print(mediainfo('/home/cst/ws_releases/src/bio_asr/data/pretrained_model_checkpoints/example_vad.wav'))
#boundaries = VAD.get_speech_segments("speechbrain/vad-crdnn-libriparty/example_vad.wav")
boundaries = VAD.get_speech_segments(audio_file)

# Print the output
VAD.save_boundaries(boundaries)
print(boundaries)
