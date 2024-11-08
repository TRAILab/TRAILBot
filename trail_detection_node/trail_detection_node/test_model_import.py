import os
import torch
# import sys
# sys.path.append('/home/trailbot/trail_ws/src/TRAILBot/trail_detection_node/trail_detection_node')
# from model_loader import FCN8s, PSPNet, LEDNet
# from torchvision import transforms
# import os

# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# def load_model(device):
#         model = LEDNet(nclass=2, backbone='resnet50', pretrained_base=True)
#         model_location = 'lednet_resnet50_trails_best_model.pth' # previously: psp_resnet50_pascal_voc_best_model
#         if os.path.isfile(f'~/.torch/models/{model_location}'):
#             print("file exists")
#             model.load_state_dict(torch.load(f'~/.torch/models/{model_location}',map_location=torch.device('cuda:0')))
#             # model.load_state_dict(torch.load(f'src/TRAILBot/trail_detection_node/trail_detection_node/model/{model_location}',map_location=torch.device('cuda:0')))
#         else:
#             print("Could not load model")
#             return None
#         model = model.to(device)
#         model.eval()
#         print('Finished loading model!')

#         return model

if __name__ == '__main__':
    model_location = 'lednet_resnet50_trails_best_model.pth' # previously: psp_resnet50_pascal_voc_best_model
    full_path = os.path.expanduser(f'~/.torch/models/{model_location}')
    if os.path.isfile(full_path):
        print("exists")
    else:
        print("no exists")
    # load_model(device)