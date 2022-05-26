import argparse, os
import torch
from torch.autograd import Variable
from scipy.ndimage import imread
from PIL import Image
import numpy as np
import time, math
import matplotlib.pyplot as plt
import sys

parser = argparse.ArgumentParser(description="PyTorch VDSR Demo")
parser.add_argument("--cuda", action="store_true", help="use cuda?")
parser.add_argument("--model", default="", type=str, help="model path")
parser.add_argument("--image", default="", type=str, help="image name")
parser.add_argument("--opt.outputDirectory", default="", type=str, help="Saving image path")
parser.add_argument("--scale", default=4, type=int, help="scale factor, Default: 4")
parser.add_argument("--gpus", default="0", type=str, help="gpu ids (default: 0)")

def AddMargin(input_float, caffe_input_size):
    h, w = input_float.shape
    newh = h 
    neww = w
    if h == 240 and w == 416:
        newh = 256
        neww = 448
    elif h == 480 and w == 832:
        newh = 480
        neww = 832
    elif h == 720 and w == 1280:
        newh = 736
        neww = 1312
    else:
        newh = 1088
        neww = 1952
    padh = int((newh - h)/2)
    padw = int((neww - w)/2)
    print(padh, padw)
    new_im = np.zeros((newh, neww), dtype=np.float64)
    tmp_im = np.zeros((h , neww), dtype=np.float64)

    for j in range(neww):
        if j < padw:
            tmp_im[:,j] = input_float[:,0]
        elif j > w+padw-1:
            tmp_im[:,j] = input_float[:,w-1]
        else: ##j >= padw && j<=padw+w-1
            tmp_im[:,j] = input_float[:,j-padw]            
            
    for i in range(newh):
        if i < padh:
            new_im[i,:] = tmp_im[0,:]
        elif i > h+padh-1:
            new_im[i,:] = tmp_im[h-1,:]
        else :
            new_im[i,:] = tmp_im[i-padh,:]
    return new_im,padh,padw
  
        

opt = parser.parse_args()
modellink = opt.model
'''  #one of the good result in Half-pixel
if "q22" in opt.image:
    modellink = "./DCTIF_or_HPel27small/model_epoch_75.pth"
if "q27" in opt.image:
    modellink = "./DCTIF_or_HPel27small/model_epoch_75.pth"
if "q32" in opt.image:
    modellink = "./DCTIF_or_HPel32small/model_epoch_45.pth" 
if "q37" in opt.image:
    modellink = "./DCTIF_or_HPel37small/model_epoch_50.pth"
'''
    #one of the good result in B
if "q22" in opt.image:
    modellink = "./Ymodel_epoch_QP22.pth" 
if "q27" in opt.image:
    modellink = "./Ymodel_epoch_QP27.pth"
if "q32" in opt.image:
    modellink = "./Ymodel_epoch_QP32.pth" 
if "q37" in opt.image:
    modellink = "./Ymodel_epoch_QP37.pth"

if "Chroma" in opt.image: 
    if "q22" in opt.image:
        modellink = "./model/UVmodel_epoch_QP22.pth"  
    if "q27" in opt.image:
        modellink = "./model/UVmodel_epoch_QP27.pth"   
    if "q32" in opt.image:
        modellink = "./model/UVmodel_epoch_QP32.pth"    
    if "q37" in opt.image:
        modellink = "./model/UVmodel_epoch_QP37.pth"  


im_input =np.loadtxt(opt.image);
#print (im_input.shape)

im_input = im_input/255
orhei, orwid = im_input.shape
im_input = Variable(torch.from_numpy(im_input).float()).view(1, -1, im_input.shape[0], im_input.shape[1])
 
os.environ["CUDA_VISIBLE_DEVICES"] = '1'
model = torch.load(modellink, map_location=lambda storage, loc: storage)["model"]
model = model.cuda()

im_input = im_input.cuda() 
out = model(im_input)
out = out.cpu()

im_h_y = out.data[0,0].numpy().astype(np.float32)*255
im_h_y[im_h_y > 255] = 255
im_h_y[im_h_y < 0] = 0


find_slash = opt.image.rfind('/')
savename  = opt.outputDirectory + opt.image[find_slash+1:None]
np.savetxt(savename, im_h_y,fmt='%i', delimiter='\n', newline='\n', )

