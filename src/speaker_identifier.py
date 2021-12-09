#!/usr/bin/env python3

import rospy
import rospkg
import os
from bio_asr.srv import SpeakerRecognitionOnFile, SpeakerRecognitionOnFileRequest, SpeakerRecognitionOnFileResponse
from bio_asr.msg import AudioFileNotification

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

        rp = rospkg.RosPack()
        path = rp.get_path("bio_asr")
        self.data_dir = os.path.join(path, "data")
        model_dir = os.path.join(path, "pretrained_models")

        # transcription clutters symlinks everywhere if we aren't in this dir
        os.chdir(self.data_dir)

        self.classifier = EncoderClassifier.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb"
        )
        self.verification = SpeakerRecognition.from_hparams(
            source="speechbrain/spkrec-ecapa-voxceleb",
            savedir=os.path.join(model_dir, "spkrec-ecapa-voxceleb"),
        )

        audio_file_use_topic = "audio_file_updates"
        self.pub_file_use = rospy.Publisher(
            audio_file_use_topic, AudioFileNotification, queue_size=1
        )

    def analyze_utterance(self, req:SpeakerRecognitionOnFileRequest):
        assert type(req.file) is str
        print('dealing with '+str(req.file))
        score, prediction = self.verification.verify_files(
            req.file,
            os.path.join(self.data_dir, 'chris.mp3')
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
