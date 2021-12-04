#!/usr/bin/env python3

import rospy
import rospkg
import os
from bio_asr.srv import VadOnFile, VadOnFileRequest, VadOnFileResponse
from bio_asr.msg import AudioFileNotification
from speechbrain.pretrained import VAD


class VadProvider:
    def __init__(self):
        rospy.loginfo('prepping')
        rp = rospkg.RosPack()
        path = rp.get_path("bio_asr")
        data_dir = os.path.join(path, "data")
        model_dir = os.path.join(path, "pretrained_models")

        # transcription clutters symlinks everywhere if we aren't in this dir
        os.chdir(data_dir)
        self.vad_model = VAD.from_hparams(
            source="speechbrain/vad-crdnn-libriparty",
            savedir=os.path.join(model_dir,"vad-crdnn-libriparty")
        )

        rospy.Service("vad_on_file", VadOnFile, self.provide_asr_on_file)
        rospy.loginfo('ready')

    def provide_asr_on_file(self, req: VadOnFileRequest):
        assert type(req.file_path) is str
        boundaries = self.vad_model.get_speech_segments(req.file_path)
        print(boundaries)
        return VadOnFileResponse()


def main():
    rospy.init_node("vad_provider", anonymous=False)
    vp = VadProvider()
    rospy.spin()


if __name__ == "__main__":
    main()
