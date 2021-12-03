#!/usr/bin/env python3

import rospy
import tempfile
from audio_common_msgs.msg import AudioData, AudioInfo
from std_srvs.srv import Empty, EmptyResponse

import torchaudio
from speechbrain.pretrained import EncoderClassifier
from speechbrain.pretrained import SpeakerRecognition



class SpeakerIdentifier(object):
    def __init__(self):
        self.sub_audio = rospy.Subscriber("/audio", AudioData, self.append_audio)
        self.sub_audio_info = rospy.Subscriber(
            "/audio_info", AudioInfo, self.receive_audio_info
        )
        self.srv_analyze_utterance = rospy.Service(
            "analyze_utterance", Empty, self.analyze_utterance
        )

        self.current_utterance = []
        self.audio_info = None

        self.classifier = EncoderClassifier.from_hparams(source="speechbrain/spkrec-ecapa-voxceleb")
        self.verification = SpeakerRecognition.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb",
            savedir="pretrained_models/spkrec-ecapa-voxceleb",
        )

    def analyze_utterance(self, req):
        del req

        assert self.audio_info is not None
        tmpfile = tempfile.TemporaryFile(suffix='.'+str(self.audio_info.coding_format))
        tmpfile.writelines(self.current_utterance)
        self.current_utterance = []

        score, prediction = self.verification.verify_files(
            tmpfile,
            "cst_test2.wav",
        )

        return EmptyResponse()

    def append_audio(self, msg):
        self.current_utterance.append(msg)

    def receive_audio_info(self, msg):
        self.audio_info = msg


def main():
    pass


if __name__ == "__main__":
    main()
