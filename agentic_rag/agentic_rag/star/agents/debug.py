import os
from PIL import Image, ImageDraw, ImageFont

def _safe_open_rgb(p):
                return Image.open(p).convert("RGB") if os.path.exists(p) else None

def _quad_no_resize(images, labels=None, pad=8, bg=(20, 20, 20), align="top"):
                """
                Place up to four images into a 2x2 grid without resizing:
                [0] [1]
                [2] [3]
                Layout adapts to the number of input images (1..4). Missing cells are filled with bg.

                Args:
                    images (list[Image.Image or None]): length 1..4. Extra items are ignored.
                    labels (list[str] or None): optional per-tile labels (len <= 4), drawn in top-left.
                    pad (int): spacing between tiles (both horizontal and vertical).
                    bg (tuple[int,int,int]): background color for canvas and empty tiles.
                    align (str): vertical alignment for each tile within its row box. One of {"top","center","bottom"}.

                Returns:
                    PIL.Image.Image: composed 2x2 canvas.
                """
                from statistics import median
                # Normalize inputs
                imgs = list(images[:4])  # max 4
                n = len(imgs)
                if n == 0:
                    # return a simple blank 2x2 canvas
                    return Image.new("RGB", (640*2 + pad, 480*2 + pad), bg)

                # Gather sizes of provided images to derive a sensible placeholder size
                sizes = [(im.width, im.height) for im in imgs if isinstance(im, Image.Image)]
                if sizes:
                    # Use median dimensions among provided images for placeholders
                    ph_w = int(median([w for w, _ in sizes]))
                    ph_h = int(median([h for _, h in sizes]))
                else:
                    ph_w, ph_h = 640, 480  # fallback when all are None

                # Replace None with placeholder tiles
                for i in range(n):
                    if imgs[i] is None:
                        imgs[i] = Image.new("RGB", (ph_w, ph_h), bg)

                # If fewer than 4 images, fill remaining cells with bg tiles
                while len(imgs) < 4:
                    imgs.append(Image.new("RGB", (ph_w, ph_h), bg))

                # Ensure all images are RGB (avoid mode mismatches)
                imgs = [im.convert("RGB") for im in imgs]

                # Compute per-column max width and per-row max height (no resizing)
                # Layout indices: 0 1 (row 0), 2 3 (row 1)
                col0_w = max(imgs[0].width, imgs[2].width)
                col1_w = max(imgs[1].width, imgs[3].width)
                row0_h = max(imgs[0].height, imgs[1].height)
                row1_h = max(imgs[2].height, imgs[3].height)

                # Canvas size
                W = col0_w + pad + col1_w
                H = row0_h + pad + row1_h
                canvas = Image.new("RGB", (W, H), bg)

                # Helper: compute y offset based on alignment within a row box
                def y_offset(box_h, img_h):
                    if align == "center":
                        return (box_h - img_h) // 2
                    elif align == "bottom":
                        return box_h - img_h
                    return 0  # "top"

                # Paste positions
                # Top-left (cell 0)
                x0, y0 = 0, 0
                y0 += y_offset(row0_h, imgs[0].height)
                canvas.paste(imgs[0], (x0, y0))

                # Top-right (cell 1)
                x1, y1 = col0_w + pad, 0
                y1 += y_offset(row0_h, imgs[1].height)
                canvas.paste(imgs[1], (x1, y1))

                # Bottom-left (cell 2)
                x2, y2 = 0, row0_h + pad
                y2 += y_offset(row1_h, imgs[2].height)
                canvas.paste(imgs[2], (x2, y2))

                # Bottom-right (cell 3)
                x3, y3 = col0_w + pad, row0_h + pad
                y3 += y_offset(row1_h, imgs[3].height)
                canvas.paste(imgs[3], (x3, y3))

                # Optional labels
                if labels:
                    draw = ImageDraw.Draw(canvas)
                    try:
                        font = ImageFont.load_default()
                    except Exception:
                        font = None
                    label_positions = [
                        (x0 + 8, (0 if align == "top" else y0) + 8),
                        (x1 + 8, (0 if align == "top" else y1) + 8),
                        (x2 + 8, (row0_h + pad if align == "top" else y2) + 8),
                        (x3 + 8, (row0_h + pad if align == "top" else y3) + 8),
                    ]
                    for i, text in enumerate(labels[:4]):
                        draw.text(label_positions[i], str(text), fill=(255, 255, 255), font=font)

                # If user provided fewer than 4 images, we keep bg tiles in remaining cells.
                # If you prefer duplicating last real image instead of bg, replace the while-fill above.

                return canvas

path1 = "/home/trailbot/RAG/results/2025-09-21 13:27:52/annotated_rgb/annotated_rgb_9.png"
path2 = "/home/trailbot/RAG/results/2025-09-21 13:27:52/annotated_rgb/annotated_rgb_10.png" 

img0 = _safe_open_rgb(path1)
img1 = _safe_open_rgb(path2)