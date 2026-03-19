"""
2024.01.16 
得到图像mask和caption的一个类MyAutomaticMaskGenerator
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
import torch
from typing import List, Dict, Optional, Any
from tokenize_anything import model_registry
from tokenize_anything.utils.image import im_rescale
from tokenize_anything.utils.image import im_vstack
import time
import torchvision
import os
import sys

# sys.path.append("/home/trailbot/trail_ws/src/TRAILBot/object_nav/")
# Add the parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from PIL import Image
# sys.path.append("/home/dyn/multimodal/Grounded-Segment-Anything")
# sys.path.append("/home/dyn/multimodal/Grounded-Segment-Anything/Tag2Text")
sys.path.append("/home/trailbot/Documents/third_parties/GroundingDINO")
sys.path.append("/home/trailbot/Documents/third_parties/Tag2Text")
sys.path.append("third_parties")
THIRD_PARTIES = os.path.expanduser(
    "~/trail_ws/src/TRAILBot/object_nav/object_nav/third_parties"
)
if THIRD_PARTIES not in sys.path:
    sys.path.insert(0, THIRD_PARTIES)

try:
    from Tag2Text.models import tag2text
    # from ram.models import tag2text
    from recognize_anything import inference_tag2text, inference_ram
    import torchvision.transforms as TS
    # print("Tag2text sub-package found. and recognize_anything found!!!!!!!!!!!!!")
    # from Tag2Text.models import tag2text
    # from Tag2Text import inference_tag2text, inference_ram
except ImportError as e:
    print("Tag2text sub-package not found. Please check your PATH. ")
    raise e

try: 
    from groundingdino.util.inference import Model
except ImportError as e:
    print("Import Error: Please install Grounded Segment Anything following the instructions in README.")
    raise e

class MyAutomaticMaskGenerator:
    
    # 初始化，把参数传入进来
    def __init__(self, tagging_model, grounding_dino_model, tap_model, sbert_model):
        self.tagging_model = tagging_model
        self.grounding_dino_model = grounding_dino_model
        self.tap_model = tap_model
        self.sbert_model = sbert_model
        # Tag2Text用到的变换
        self.tagging_transform = TS.Compose([
            TS.Resize((384, 384)),
            TS.ToTensor(), 
            TS.Normalize(mean=[0.485, 0.456, 0.406],
                            std=[0.229, 0.224, 0.225]),
        ])
        
        self.specified_tags='None'
        self.classes = None
        # 一些增加和删减的类别
        self.add_classes = ["other item","pavement","grass","house","bicycle","motorcycle","person","parking",
                            "fence","sidewalk","tree","vegetation","sign","building","bush","rail","pole"]
        # self.remove_classes = ["modern"]
        self.remove_classes = [
            "room", "kitchen", "office", "home", "corner",
            "shadow", "carpet", "photo", "shade", "stall", "space", "aquarium", 
            "image", "city", "blue", "skylight", "hallway", 
            "modern", "salon", "doorway", "wall lamp"
        ]
        
    @torch.no_grad()
    def generate(self, image: np.ndarray, save_path: str = None, save_vis: bool = True) -> List[Dict[str, Any]]:
        
        # start_time = time.time()
        #####################################################
        ############## 一、使用tag2text先生成标签 ############
        #####################################################
        image_rgb = image#cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # 1.1 图像预处理
        image_pil = Image.fromarray(image_rgb)
        raw_image = image_pil.resize((384, 384))
        raw_image = self.tagging_transform(raw_image).unsqueeze(0).to("cuda")
        # 1.2 图像输入进模型推理
        res = inference_tag2text.inference(raw_image , self.tagging_model, self.specified_tags)
        # 1.3 得到结果，并设定一些需要的classes
        caption=res[2]
        
        text_prompt=res[0].replace(' |', ',')
        
        classes = self.process_tag_classes(
            text_prompt, 
            add_classes = self.add_classes,
            remove_classes = self.remove_classes,
        )
        # end_time = time.time()
        # execution_time = end_time - start_time
        # print(f"tag2text Model output time: {execution_time} seconds")
        
        #####################################################
        ############## 二、使用dino为标签生成边界框 ############
        #####################################################
        # 2.1 模型推理
        BGR_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        detections = self.grounding_dino_model.predict_with_classes(
            image=BGR_img, # This function expects a BGR image...
            classes=classes,
            box_threshold=0.25,
            text_threshold=0.25,
        )
        # 2.2 非极大值抑制和-1类别，删除一部分
        if len(detections.class_id) > 0:
            # print(f"Before NMS: {len(detections.xyxy)} boxes")
            nms_idx = torchvision.ops.nms(
                torch.from_numpy(detections.xyxy), 
                torch.from_numpy(detections.confidence), 
                0.5
            ).numpy().tolist()
            # print(f"After NMS: {len(detections.xyxy)} boxes")
            detections.xyxy = detections.xyxy[nms_idx]
            detections.confidence = detections.confidence[nms_idx]
            detections.class_id = detections.class_id[nms_idx]
            # 删掉类别为-1的
            valid_idx = detections.class_id != -1
            detections.xyxy = detections.xyxy[valid_idx]
            detections.confidence = detections.confidence[valid_idx]
            detections.class_id = detections.class_id[valid_idx]
            
        # end_time = time.time()
        # execution_time = end_time - start_time
        # print(f"dino Model output time: {execution_time} seconds")
        
        
        #####################################################
        ########## 三、使用tap为边界框生成mask和caption ########
        #####################################################
        # 3.1 图像预处理
        vis_img = image.copy()[:, :, ::-1]
        img_list, img_scales = im_rescale(image, scales=[1024], max_size=1024)
        input_size, original_size = img_list[0].shape, image.shape[:2]
        img_batch = im_vstack(img_list, fill_value=self.tap_model.pixel_mean_value, size=(1024, 1024))
        inputs = self.tap_model.get_inputs({"img": img_batch})
        inputs.update(self.tap_model.get_features(inputs))
        # 3.2 根据上面的mask转化需要的格式
        batch_points = np.zeros((len(detections.xyxy), 2, 3), dtype=np.float32)
        for i in range(len(detections.xyxy)):
            batch_points[i, 0, :2] = detections.xyxy[i, :2]
            batch_points[i, 1, :2] = detections.xyxy[i, 2:]
            batch_points[i, 0, 2] = 2
            batch_points[i, 1, 2] = 3
        inputs["points"] = batch_points
        inputs["points"][:, :, :2] *= np.array(img_scales, dtype="float32")
        # 3.3 模型开始推理，得到mask大小
        outputs = self.tap_model.get_outputs(inputs)
        iou_pred = outputs["iou_pred"].cpu().numpy()
        point_score = batch_points[:, 0, 2].__eq__(2).__sub__(0.5)[:, None]
        rank_scores = iou_pred + point_score * ([1000] + [0] * (iou_pred.shape[1] - 1))
        mask_index = np.arange(rank_scores.shape[0]), rank_scores.argmax(1)
        mask_pred = outputs["mask_pred"]
        masks = mask_pred[mask_index]
        masks = self.tap_model.upscale_masks(masks[:, None], img_batch.shape[1:-1])
        masks = masks[..., : input_size[0], : input_size[1]]
        masks = self.tap_model.upscale_masks(masks, original_size).gt(0).cpu().numpy()
        # 3.4 模型继续推理，得到conceptscaptions
        # 推理concepts
        concepts, scores = self.tap_model.predict_concept(outputs["sem_embeds"][mask_index])
        concepts, scores = [x for x in (concepts, scores)]
        # 推理captions
        sem_tokens = outputs["sem_tokens"][mask_index]#.unsqueeze_(1)
        captions = self.tap_model.generate_text(sem_tokens)
        # captions = [x for x in captions]
        # print(captions)
        caption_fts = self.sbert_model.encode(captions, convert_to_tensor=True, device="cuda")
        caption_fts = caption_fts / caption_fts.norm(dim=-1, keepdim=True)
        
        # end_time = time.time()
        # execution_time = end_time - start_time
        # print(f"tap Model output time: {execution_time} seconds")
        # print(concepts)
        # print(scores)
        # print(captions)
        print(f"Number of masks generated: {len(masks)}, Number of concepts: {len(concepts)}, Number of captions: {len(captions)}, length of caption_fts: {len(caption_fts)}")
        anotatated_image =self.prepare_labeled_contours(image, masks, detections, image.shape[1], image.shape[0])
        #####################################################
        ########## 四、最后的可视化并保存结果 ########
        #####################################################
        if save_vis:
            plt.figure(figsize=(20,8))
            plt.imshow(vis_img)
            self.show_masks(masks, concepts, captions, plt.gca(), detections)
            plt.axis('off')
            # 如果提供了保存路径，则保存图像
            if save_path:
                # 如果目录不存在则创建
                save_path.parent.mkdir(parents=True, exist_ok=True)
                # 获取图像中非零像素的边界框
                non_zero_pixels = cv2.findNonZero(cv2.cvtColor(vis_img, cv2.COLOR_BGR2GRAY))
                x, y, w, h = cv2.boundingRect(non_zero_pixels)
                # 裁剪图像
                cropped_img = vis_img[y:y+h, x:x+w]
                # 保存裁剪后的图像
                plt.savefig(save_path, bbox_inches='tight', pad_inches=0)
                plt.close()  # 关闭图形，防止显示在 notebook 中
                
        # 在循环中，对于每个 mask，根据 detections 获取相应的边界框坐标，并保存原始图像
        # for i, (mask, concept, caption, caption_ft) in enumerate(zip(masks, concepts, captions, caption_fts)):
        #     # 找到 mask 中 True 值的坐标
        #     true_coords = np.argwhere(mask)
        #     if len(true_coords) > 0:
        #         # 根据 detections 获取相应的边界框坐标
        #         box = detections.xyxy[i]
        #         x_min, y_min, x_max, y_max = int(box[0]), int(box[1]), int(box[2]), int(box[3])
        #         # 使用边界框坐标在原始图像上裁剪相应的区域
        #         cropped_image = image_rgb.copy()
        #         # 将 mask 之外的区域填充为白色
        #         mask = mask[0]
        #         cropped_image[~mask] = [255, 255, 255]  # 白色的 RGB 值
        #         cropped_image = cropped_image[y_min:y_max, x_min:x_max]
        #         # 将裁剪的图像保存到磁盘上
        #         mask_image_path = save_path.parent / f"mask_{i+1}_image.jpg"
        #         cv2.imwrite(str(mask_image_path), cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
        #         print(caption)
        #         print(f"Mask {i+1} image saved at: {mask_image_path}")

        # 返回结果
        result = []
        
        for i, (mask, concept, caption, caption_ft) in enumerate(zip(masks, concepts, captions, caption_fts)):
            
            bbox = detections.xyxy[i].astype(int).tolist()
            result.append({
                "mask": mask,
                "concepts": concept,
                "caption": caption,
                "caption_ft": caption_ft.cpu(),
                #"img_bbox1": bbox,
                "img_bbox": detections.xyxy[i],
            })
        return result, anotatated_image
    

    def process_tag_classes(self, text_prompt:str, add_classes:List[str]=[], remove_classes:List[str]=[]) -> list[str]:
        '''
        将 Tag2Text 中的文本提示转换为类列表,方便dino使用
        '''
        classes = text_prompt.split(',')
        classes = [obj_class.strip() for obj_class in classes]
        classes = [obj_class for obj_class in classes if obj_class != '']
        for c in add_classes:
            if c not in classes:
                classes.append(c)
        for c in remove_classes:
            classes = [obj_class for obj_class in classes if c not in obj_class.lower()]
        return classes
    
    def prepare_labeled_contours(self, image, masks, detections, image_width, image_height):
            """
            Display object contours with numbered labels, dynamically positioning labels based on contour size.

            Args:
                image (np.ndarray): The original RGB image as a NumPy array.
                masks (list of np.ndarray): Binary masks for each object.
                detections (list or np.ndarray): Bounding boxes with coordinates [x_min, y_min, x_max, y_max].
                image_width (int): Width of the image.
                image_height (int): Height of the image.

            Returns:
                np.ndarray: Annotated RGB image with contours and labels drawn.
            """
            # Define distinct colors for each object
            distinct_colors = [
                '#A52A2A', '#5F9EA0', '#D2691E', '#9ACD32', '#DA70D6', '#7FFFD4',
                '#FF8000', '#8000FF', '#0080FF', '#80FF00', '#FF0080', '#00FF80',
                '#FF0000', '#00FF00', '#0000FF', '#FF00FF', '#00FFFF', '#FFFF00',
                '#FF4500', '#2E8B57'
            ]

            # Convert color hex to RGB for OpenCV
            distinct_colors_rgb = [
                tuple(int(color.lstrip('#')[i:i + 2], 16) for i in (0, 2, 4)) for color in distinct_colors
            ]

            # Copy the original image for annotation
            annotated_image = image.copy()

            # Track label positions to prevent overlap
            label_positions = []
            offset_step = 5  # Offset distance for small contours

            def clamp(value, min_value, max_value):
                """Clamp a value to ensure it stays within min and max bounds."""
                return max(min_value, min(value, max_value))

            def find_safe_label_position(contour, bbox, is_small):
                """Find a safe label position inside or near the object contour."""
                x_min, y_min, x_max, y_max = bbox
                bbox_center = (int((x_min + x_max) / 2), int((y_min + y_max) / 2))

                if not is_small:
                    # Try placing label inside the contour (center)
                    return bbox_center

                # If small, find a nearby position
                return (
                    clamp(x_max, 0, image_width),
                    clamp(y_min - offset_step, 0, image_height)
                )

            # Iterate through each detection and mask
            label_dict = {}
            for i, (mask, box) in enumerate(zip(masks, detections.xyxy)):
                color = distinct_colors_rgb[i % len(distinct_colors_rgb)]  # Cycle through colors
                # Validate mask (Ensure it's not empty or incorrectly shaped)
                if mask is None or mask.size == 0:
                    print(f"Warning: Empty mask detected at index {i}")
                    continue
                
                # Convert mask to uint8 format
                mask_uint8 = (mask.astype(np.uint8) * 255)
                # Ensure mask is 2D (Remove extra channel if present)
                if mask_uint8.ndim == 3 and mask_uint8.shape[0] == 1:
                    mask_uint8 = mask_uint8.squeeze(0)  # Remove the first dimension
                
                # Find contours
                contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # If no valid contour, skip
                if not contours:
                    print(f"Warning: No contours found for mask at index {i}")
                    continue

                # Determine if the contour is small
                x_min, y_min, x_max, y_max = map(int, box)
                bbox_width = x_max - x_min
                bbox_height = y_max - y_min
                is_small = bbox_width < 30 or bbox_height < 30

                # Draw contours on the image
                cv2.drawContours(annotated_image, contours, -1, color, 2)

                # Find a safe label position
                label_pos_x, label_pos_y = find_safe_label_position(contours[0], (x_min, y_min, x_max, y_max), is_small)

                # Define label text
                label_text = f"{i + 1}"
                font_size = 0.35
                #label_size, _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, font_size, 1)
                label_width, label_height = 14, 14

                # Adjust label position to prevent out-of-bounds issues
                # label_pos_x = clamp(label_pos_x, 0, image_width - label_width)
                # label_pos_y = clamp(label_pos_y, label_height + 4, image_height)
                # Store the new label position
                label_positions.append((label_pos_x, label_pos_y))
                label_dict[label_text] = (label_pos_x, label_pos_y - label_height, label_pos_x + label_width, label_pos_y, color, label_pos_x, label_pos_y-3)
                # Draw label background rectangle
            for key in label_dict.keys():
                cv2.rectangle(
                    annotated_image,
                    (label_dict[key][0], label_dict[key][1]),
                    (label_dict[key][2], label_dict[key][3]),
                    label_dict[key][4],
                    -1  # Filled rectangle
                )
                cv2.putText(
                    annotated_image,
                    key,
                    (label_dict[key][5], label_dict[key][6]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_size,
                    (0, 0, 0),  # Black text
                    1
                )
                # cv2.rectangle(
                #     annotated_image,
                #     (label_pos_x, label_pos_y - label_height - 4),
                #     (label_pos_x + label_width, label_pos_y),
                #     color,
                #     -1  # Filled rectangle
                # )

                # # Put label text
                # cv2.putText(
                #     annotated_image,
                #     label_text,
                #     (label_pos_x, label_pos_y - 2),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     font_size,
                #     (0, 0, 0),  # Black text for contrast
                #     1
                # )

            return annotated_image
    
    def show_masks(self, masks, concepts, captions, ax, detections):
        '''
        保存图像可视化用
        '''
        for i, (mask, concept, caption) in enumerate(zip(masks, concepts, captions)):
            # 找到mask中True值的坐标
            true_coords = np.argwhere(mask)
            if len(true_coords) > 0:
                # 显示mask
                color = np.concatenate([np.random.random(3), np.array([1])], axis=0)  # 调整颜色透明度
                ax.imshow(mask.reshape(mask.shape[-2:] + (1,)) * color.reshape(1, 1, -1), alpha=0.9, label=f'Mask {i+1}')
                # 展示box
                box = detections.xyxy[i]
                rect = plt.Rectangle((box[0], box[1]), box[2] - box[0], box[3] - box[1], linewidth=1.5, edgecolor='r', facecolor='none')
                ax.add_patch(rect)
                # 展示文字
                center_x = (box[0]+box[2])/2
                center_y = (box[1]+box[3])/2
                caption_width = len(caption) * 5  # 根据字体大小调整
                caption_x = center_x - caption_width / 2
                caption_y = center_y
                # 显示caption
                ax.text(caption_x, caption_y, f"{concept}:{caption}", color='black', fontsize=8, bbox=dict(facecolor=color, alpha=1.0))
