import dill
import pygame
import json
from pymunk.pygame_util import DrawOptions
import os,shutil
from datetime import datetime
import zipfile
from .PM_replayable_core import PMReplayableCore
from ..utils.analytics import get_str_analytics


class PMReplayableExporter(PMReplayableCore):
    """
    Create a zip file with rendered video of the path,analytics and conifguration
    """
    def __init__(self, replay_file):
        super().__init__(replay_file)
        self.cnt = 0
        self.pic_dir = "/tmp/pygame_frames"
        if os.path.exists(self.pic_dir):
            shutil.rmtree(self.pic_dir)
        os.makedirs(self.pic_dir)

    def render(self,destination_folder,constraints=False):
        print("START RENDERING")
        self.sim.import_from(self.path[0].sim_export)
        if self.one_time_draw is not None:
            self.one_time_draw(self.sim, self.one_time_canvas, self.additional_data)

        draw_ops = DrawOptions(self.cur_scene)
        if not constraints:
            draw_ops.flags = DrawOptions.DRAW_SHAPES

        for n in self.path[1:]:
            parent = n.replayer.parent
            print("Iterating now: ", n.replayer.segment_iter_cnt)
            for i in range(n.replayer.segment_iter_cnt):
                if self._onestep(parent, n, i,draw_ops):
                    break
                fullpath = os.path.join(self.pic_dir, f"frame_{self.cnt}.png")
                pygame.image.save(self.cur_scene, os.path.join(self.pic_dir, fullpath))
                self.cnt += 1

        print("Images saved, now converting to video")
        self._render(destination_folder)
        shutil.rmtree(self.pic_dir)



    def _render(self,destination_folder):
        path = os.path.join(destination_folder, "video.mp4")
        os.system(f"ffmpeg -framerate {self.fps} -i {self.pic_dir}/frame_%d.png -c:v libx264 -pix_fmt yuv420p -y {path}")
        print("Video saved")

    def export_analytics(self,destination_folder):
        analytics = self.additional_data.get("analytics",None)
        steps = self.additional_data.get("steps",None)
        if analytics is None:
            print("No analytics provided")
            return

        path = os.path.join(destination_folder, "analytics.json")
        with open(path, "w+") as f:
            json.dump(analytics,f,indent=4)

        print("Analytics saved")

    def export_config(self,destination_folder):
        config = self.additional_data.get("config",None)
        if config is None:
            print("No config provided")
            return

        path = os.path.join(destination_folder,"config.json")
        config.save_to_file(path)
        print("Config saved")
    def export_path_file(self,destination_folder):
        path = os.path.join(destination_folder,"path.pkl")
        with open(path,"wb") as f:
            dill.dump(self.path,f)
        print("Path saved")

    def save_all(self,destination_folder,constraints=False):
        destination_folder = os.path.join(destination_folder,"replayable_export")
        if not os.path.exists(destination_folder):
            os.makedirs(destination_folder)
        self.render(destination_folder,constraints)
        # self._render(destination_folder)
        self.export_analytics(destination_folder)
        self.export_config(destination_folder)
        self.export_path_file(destination_folder)
        print("All saved")

    def save_all_zip(self,destination_folder,constraints=False):
        self.save_all(destination_folder,constraints)
        current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        zip_name = f"{current_time}.zip"
        zip_path = os.path.join(destination_folder,zip_name)
        with zipfile.ZipFile(zip_path, 'w') as zipf:
            # Walk through the folder and add each file to the zip
            for root, dirs, files in os.walk(os.path.join(destination_folder,"replayable_export")):
                for file in files:
                    zipf.write(os.path.join(root, file), os.path.relpath(os.path.join(root, file), os.path.join(destination_folder,"replayable_export")))
