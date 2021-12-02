#!/usr/bin/env python3

from speechbrain.pretrained import EncoderDecoderASR
from time import time

asr_model = EncoderDecoderASR.from_hparams(
    source="speechbrain/asr-crdnn-rnnlm-librispeech",
    savedir="pretrained_models/asr-crdnn-rnnlm-librispeech",
)

t = time()
transcription = asr_model.transcribe_file("cst_test1.wav")
print(time() - t)
print(transcription)
t = time()
transcription = asr_model.transcribe_file("cst_test2.wav")
print(time() - t)
print(transcription)
