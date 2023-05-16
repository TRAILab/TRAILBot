based on https://www.tensorflow.org/hub/tutorials/movenet


pip installs:
```
pip install -q imageio
pip install -q opencv-python
pip install -q git+https://github.com/tensorflow/docs
pip install tensorflow_hub tensorflow ipython
```


## current progress
| model | FPS (r7-5800u CPU) | FPS (GPU) | doesn't break with multi-person | ok with cropped people? |
| -------- | -------- |  -------- | -------- | -------- |
| movenet_lightning_f16_tflite | 62 | ? | yes | yes | 
| movenet_thunder_f16_tflite | 16 | ? | yes | yes |
| movenet_lightning_int8_tflite | 76 | ? | broken | somewhat |
| movenet_thunder_int8_tflite | 28 | ? | broken | yes |
| movenet_lightning tfhub |55 | ? | yes (somewhat)  | yes |
| movenet_thunder tfhub |27 | ? | yes  | yes |
