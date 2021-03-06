#!/usr/bin/env python3

import rospy
import rospkg
import os
from torch import Tensor
from bio_asr.srv import VadOnFile, VadOnFileRequest, VadOnFileResponse
from bio_asr.msg import AudioFileNotification
from rospy.rostime import Time
from speechbrain.pretrained import VAD

class VadProvider:
    def __init__(self):
        rospy.loginfo("prepping")
        rp = rospkg.RosPack()
        path = rp.get_path("bio_asr")
        data_dir = os.path.join(path, "data")
        model_dir = os.path.join(path, "pretrained_models")

        self.vad_model = VAD.from_hparams(
            source="speechbrain/vad-crdnn-libriparty",
            savedir=os.path.join(model_dir, "vad-crdnn-libriparty"),
        )

        rospy.Service("run_vad_on_file", VadOnFile, self.provide_asr_on_file)
        rospy.loginfo("ready")

        audio_file_use_topic = "audio_file_updates"
        self.pub_file_use = rospy.Publisher(
            audio_file_use_topic, AudioFileNotification, queue_size=1
        )
        # transcription clutters symlinks everywhere if we aren't in this dir
        os.chdir('/tmp')

    def provide_asr_on_file(self, req: VadOnFileRequest):
        assert type(req.file_path) is str
        rospy.logdebug('[provide_asr_on_file]: performing vad on ' + str(req.file_path))
        boundaries:Tensor = self.vad_model.get_speech_segments(req.file_path)
        resp = VadOnFileResponse()
        if len(boundaries) <= 0:
            resp.marks.results = [resp.marks.NON_SPEECH]
            resp.marks.timesteps = [Time(0)]
        if len(boundaries) == 1:
            resp.marks.results = [resp.marks.SPEECH]
            resp.marks.timesteps = [Time(0)]

        return resp


def main():
    rospy.init_node("vad_provider", anonymous=False)
    vp = VadProvider()
    rospy.spin()


if __name__ == "__main__":
    main()
