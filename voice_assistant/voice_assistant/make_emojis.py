# """
# This file makes emoji pngs and saves them.
# """
# from PIL import Image, ImageFont
# from pilmoji import Pilmoji

# cool = '😎'
# think = '🤔'
# listen = '👂'
# speaker = '🔊'
# mic = '🎙️'
# smile = '😊'


# with Image.new('RGB', (700, 800), (255, 255, 255)) as image:
#     font = ImageFont.truetype('Ubuntu-R.ttf', 500)

#     with Pilmoji(image) as pilmoji:
#         pilmoji.text((100, 600), 'Speaking...'.strip(),
#                      (0, 0, 0), ImageFont.truetype('Ubuntu-R.ttf', 100))
#         image.save('smile_emoji.png')


import cv2
cv2.startWindowThread()
cv2.namedWindow("GUI", cv2.WINDOW_NORMAL)
cv2.resizeWindow("GUI", 2000, 2000)
path = '/home/trailbot/trail_ws/src/TRAILBot/voice_assistant/emojis/smile_emoji.png'
img = cv2.imread(path)
cv2.imshow("GUI", img)
cv2.waitKey(0)