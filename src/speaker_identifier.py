#!/usr/bin/env python3

import rospy
import rospkg
import os
from bio_asr.srv import SpeakerRecognitionOnFile, SpeakerRecognitionOnFileRequest, SpeakerRecognitionOnFileResponse
from bio_asr.srv import GetKnownAgents, GetKnownAgentsRequest, GetKnownAgentsResponse
from bio_asr.msg import AudioFileNotification

#import torchaudio
from speechbrain.pretrained import EncoderClassifier
from speechbrain.pretrained import SpeakerRecognition


class SpeakerIdentifier(object):
    def __init__(self):
        self.srv_analyze_utterance = rospy.Service(
            "run_recog_on_file", SpeakerRecognitionOnFile, self.analyze_utterance
        )

        self.get_known_agents   = rospy.ServiceProxy('get_known_agents',  GetKnownAgents)

        self.current_utterance = []
        self.audio_info = None

        rp = rospkg.RosPack()
        path = rp.get_path("bio_asr")
        self.data_dir = os.path.join(path, "data")
        model_dir = os.path.join(path, "pretrained_models")

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

        # transcription clutters symlinks everywhere if we aren't in this dir
        os.chdir('/tmp')


    def analyze_utterance(self, req:SpeakerRecognitionOnFileRequest):
        assert type(req.file) is str
        print('dealing with '+str(req.file))
        known_agents_req = GetKnownAgentsRequest()
        known_agents:GetKnownAgentsResponse = self.get_known_agents(known_agents_req)
        assert len(known_agents.comparison_file_paths) == len(known_agents.agent_names)

        resp = SpeakerRecognitionOnFileResponse()
        resp.agent = ''
        resp.found_match = False
        max_score = 0
        for n in range(0, len(known_agents.comparison_file_paths)):
            name = known_agents.agent_names[n]
            path = known_agents.comparison_file_paths[n]
            score, prediction = self.verification.verify_files(
                req.file,
                path
            )

            if score > max_score:
                max_score = score
                resp.agent = name

            if prediction:
                resp.found_match = True
                resp.agent = name

        resp.found_match = True
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
