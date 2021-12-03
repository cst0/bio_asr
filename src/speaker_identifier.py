#!/usr/bin/env python3

import rospy
import tempfile
from std_srvs.srv import Empty, EmptyResponse
from bio_asr.srv import SpeakerRecognitionOnFile, SpeakerRecognitionOnFileRequest, SpeakerRecognitionOnFileResponse

import torchaudio
from speechbrain.pretrained import EncoderClassifier
from speechbrain.pretrained import SpeakerRecognition


class SpeakerIdentifier(object):
    def __init__(self):
        self.srv_analyze_utterance = rospy.Service(
            "run_recog_on_file", SpeakerRecognitionOnFile, self.analyze_utterance
        )

        self.current_utterance = []
        self.audio_info = None

        self.classifier = EncoderClassifier.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb"
        )
        self.verification = SpeakerRecognition.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb",
            savedir="pretrained_models/spkrec-ecapa-voxceleb",
        )

    def analyze_utterance(self, req:SpeakerRecognitionOnFileRequest):
        assert type(req.file) is str
        score, prediction = self.verification.verify_files(
            req.file,
            "/home/cst/ws_releases/src/bio_asr/data/afterapplepicking.mp3"
        )

        resp = SpeakerRecognitionOnFileResponse()
        rospy.logdebug('running speaker verification, evaluated to '+str(prediction))
        if prediction:
            resp.agent = 'chris'
            resp.found_match = True
        else:
            resp.agent = ''
            resp.found_match = False

        return resp

    def append_audio(self, msg):
        self.current_utterance.append(msg)

    def receive_audio_info(self, msg):
        self.audio_info = msg


def main():
    rospy.init_node("speaker_identifier", anonymous=False)
    si = SpeakerIdentifier()
    rospy.spin()


if __name__ == "__main__":
    main()
