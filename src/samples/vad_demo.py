#!/usr/bin/env python3

from speechbrain.pretrained import VAD

VAD = VAD.from_hparams(
    source="speechbrain/vad-crdnn-libriparty",
    savedir="pretrained_models/vad-crdnn-libriparty",
)
boundaries = VAD.get_speech_segments("speechbrain/vad-crdnn-libriparty/example_vad.wav")

# Print the output
VAD.save_boundaries(boundaries)
print(boundaries)
