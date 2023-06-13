"""
This file makes emoji pngs and saves them.
"""
from PIL import Image, ImageFont
from pilmoji import Pilmoji

cool = '😎'
think = '🤔'
listen = '👂'
speaker = '🔊'
mic = '🎙️'
smile = '😊'


with Image.new('RGB', (700, 800), (255, 255, 255)) as image:
    font = ImageFont.truetype('Ubuntu-R.ttf', 500)

    with Pilmoji(image) as pilmoji:
        pilmoji.text((100, 100), smile.strip(), (0, 0, 0), font)
        pilmoji.text((100, 600), 'Speaking...'.strip(),
                     (0, 0, 0), ImageFont.truetype('Ubuntu-R.ttf', 100))
        image.save('smile_emoji.png')
