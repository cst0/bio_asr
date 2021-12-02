#!/usr/bin/env python3

from speechbrain.pretrained import EncoderDecoderASR
from time import time
import os
import rospkg

rp = rospkg.RosPack()
path = rp.get_path('bio_asr')
data_dir = os.path.join(path, 'data')
model_dir = os.path.join(path, 'pretrained_models')

# transcription clutters symlinks everywhere if we aren't in this dir
os.chdir(data_dir)

asr_model = EncoderDecoderASR.from_hparams(
    source="speechbrain/asr-crdnn-rnnlm-librispeech",
    savedir=os.path.join(model_dir, "asr-crdnn-rnnlm-librispeech"),
)

t = time()
transcription = asr_model.transcribe_file(os.path.join(data_dir, "cst_test1.wav"))
print('that took', time() - t)
print(transcription)
t = time()
transcription = asr_model.transcribe_file(os.path.join(data_dir, "cst_test2.wav"))
print('that took', time() - t)
print(transcription)
