#!/usr/bin/env python3

import torchaudio
from speechbrain.pretrained import EncoderClassifier
from speechbrain.pretrained import SpeakerRecognition

classifier = EncoderClassifier.from_hparams(source="speechbrain/spkrec-ecapa-voxceleb")
verification = SpeakerRecognition.from_hparams(
    source="speechbrain/spkrec-ecapa-voxceleb",
    savedir="pretrained_models/spkrec-ecapa-voxceleb",
)
score, prediction = verification.verify_files(
    "cst_test1.wav",
    "cst_test2.wav",
)
print(score, prediction)
