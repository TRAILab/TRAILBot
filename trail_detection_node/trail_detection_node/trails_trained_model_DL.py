"""Prepare Trails dataset"""
import os
# import zipfile
# import shutil
import argparse
import gdown

# For Full Dataset
_TARGET_DIR = os.path.expanduser('~/.torch/models') #Directory to store data
_DRIVE_ZIP_NAME = 'lednet_resnet50_trails_best_model' #"Name of zip file"
_FILE_ID= "17aTMP9MX2qxENSBtcBUvZvRVqdL2IB_h" #Use Id for zip file on drive, ensure "Anyone with link can access"

def parse_args():
    parser = argparse.ArgumentParser(
        description='Import Trails trained model',
        epilog='Example: python3 trails_trained_model_DL.py',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--zip-name', default=_DRIVE_ZIP_NAME, help='Name of zip file')
    parser.add_argument('--drive-zip-id', default=_FILE_ID, help='ID of the drive zip file')
    args = parser.parse_args()
    return args 

def save_with_gdown(id, destination):
    url = 'https://drive.google.com/uc?id='+id
    gdown.download(url, destination, quiet=False)  



if __name__ == '__main__':
    args = parse_args()
    os.makedirs(_TARGET_DIR, exist_ok = True)
    print("~/.torch/models directory made/exists")        
    zip_path = _TARGET_DIR + '/'+ _DRIVE_ZIP_NAME +'.pth'
    os.remove(zip_path) if os.path.exists(zip_path) else None
    save_with_gdown(args.drive_zip_id, zip_path)   