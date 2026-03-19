import argparse
import importlib.util
import json
import os
import sys
# Add the parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from utils.captioner import Captioner
from PIL import Image
from pydantic import BaseModel
from termcolor import colored

from PIL import Image as PILImage
from io import BytesIO
import tempfile

import llava
from llava import conversation as clib
# from llava.media import Image, Video
from llava.media import Image as LlavaImage, Video
from llava.model.configuration_llava import JsonSchemaResponseFormat, ResponseFormat


class NVILACaptioner(Captioner):
    def __init__(self, args):
        # Convert json mode to response format
        if not args.json_mode:
            self.response_format = None
        elif args.json_schema is None:
            self.response_format = ResponseFormat(type="json_object")
        else:
            schema_str = get_schema_from_python_path(args.json_schema)
            print(schema_str)
            self.response_format = ResponseFormat(type="json_schema", json_schema=JsonSchemaResponseFormat(schema=schema_str))

        self.model = llava.load(args.model_path).to("cuda")
        # Set conversation mode
        clib.default_conversation = clib.conv_templates[args.conv_mode].copy()
        self.args = args

    def get_schema_from_python_path(self, path: str) -> str:
        schema_path = os.path.abspath(path)
        spec = importlib.util.spec_from_file_location("schema_module", schema_path)
        schema_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(schema_module)

        # Get the Main class from the loaded module
        Main = schema_module.Main
        assert issubclass(
            Main, BaseModel
        ), f"The provided python file {path} does not contain a class Main that describes a JSON schema"
        return Main.schema_json()
    
    def caption(self, images=None, query: str = None):
        # Prepare multi-modal prompt
        prompt = []
        if self.args.media is not None or images is not None:
            media_np = images if images is not None else self.args.media
            media = Image.fromarray(media_np, mode='RGB')
            idx = 0
            # ✅ 新逻辑：支持字符串、PIL、llava.media.Image
            if isinstance(media, (LlavaImage, Video)):
                pass  # already wrapped
            elif isinstance(media, str):
                if any(media.endswith(ext) for ext in [".jpg", ".jpeg", ".png"]):
                    media = LlavaImage(media)
                elif any(media.endswith(ext) for ext in [".mp4", ".mkv", ".webm"]):
                    media = Video(media)
                else:
                    raise ValueError(f"Unsupported media path: {media}")
            elif isinstance(media, PILImage.Image):
                # Save PIL image to temp file and wrap
                with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
                    media.save(tmp.name, format="JPEG")
                    media = LlavaImage(tmp.name)
            else:
                raise TypeError(f"Unsupported media input type: {type(media)}")

            prompt.append(f"the label of this image is: {idx}")
            prompt.append(media)
        # Add text input

        if query is not None:
            prompt.append(query)
        else:
            prompt.append(self.args.query)

        # Generate response
        print("\033[96mbefore\033[0m")
        response = self.model.generate_content(prompt, response_format=self.response_format)
        print("\033[96mafter\033[0m")
        return response
    # def caption(self, images=None, query: str = None):
    #     # Prepare multi-modal prompt
    #     prompt = []
    #     if self.args.media is not None or images is not None:
    #         inputs = images if images is not None else self.args.media
    #         idx = 0
    #         for media in inputs or []:

    #             if any(media.endswith(ext) for ext in [".jpg", ".jpeg", ".png"]):
    #                 media = Image(media)
    #             elif any(media.endswith(ext) for ext in [".mp4", ".mkv", ".webm"]):
    #                 media = Video(media)
    #             else:
    #                 raise ValueError(f"Unsupported media type: {media}")
    #             prompt.append(f"the label of this image is: {idx}")
    #             idx += 1
    #             print("media: ", media)
    #             prompt.append(media)

    #     if query is not None:
    #         prompt.append(query)
    #     else:
    #         prompt.append(self.args.query)
    #     print("input: ", prompt)
    #     # Generate response
    #     response = self.model.generate_content(prompt, response_format=self.response_format)
    #     print(colored(response, "cyan", attrs=["bold"]))
    #     return response
    
#     def main(self):
#         self.caption()

# # initialize the NVILACaptioner

# if __name__ == "__main__":
#     default_query = "Please describe the video in detail"#"Please describe in detail what you see from the video. Specifically focus on the people, objects, environmental features, events/ectivities, and other interesting details. Think step by step about these details and be very specific."
#     base_path = "/media/ssd/Local_data/CODa_dataset/data/2d_rect/cam0/0/"
#     img = "2d_rect_cam0_0_"
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--model-path", type=str, default="Efficient-Large-Model/NVILA-8B")
#     parser.add_argument("--conv-mode", "-c", type=str, default="auto")
#     parser.add_argument("--query", type=str, default=default_query)
#     parser.add_argument("--text", type=str)
#     # parser.add_argument("--num_video_frames", "-nf", type=str, default="16") #default="8/16/32/64/128/256/512"

#     # parser.add_argument("--media", type=str, nargs="+", default= [f"{base_path}{img}{200}.png", f"{base_path}{img}{400}.png", f"{base_path}{img}{800}.png"])#["/home/mfyuan/local_folder/OpenNav_v2/deps1/VILA/demo_images/cam0.png"])
#     parser.add_argument("--media", type=str, default=["/home/mfyuan/local_folder/OpenNav_v2/deps1/VILA/demo_images/output.mp4"])#/media/ssd/Local_data/CODa_dataset/videos/0/output.mp4
#     parser.add_argument("--json-mode", action="store_true")
#     parser.add_argument("--json-schema", type=str, default=None)
#     args = parser.parse_args()
#     captioner = NVILACaptioner(args)
#     captioner.main()