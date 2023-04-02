"""This is a script to generate a video with all demonstration sequences"""
import numpy as np
import h5py
import numpy as np
import torch
import torchvision
import imageio
import numpy as np
import math
import argparse

def generate_video_stream(imgs, fps=30, video_name="./video.mp4"):
    try:
        if "mp4" not in video_name:
            video_name = f"{video_name}.mp4"
        video_writer = imageio.get_writer(video_name, fps=fps)
        for img in imgs:
            video_writer.append_data(img)
        video_writer.close()
        return True
    except Exception as e:
        print(e)
        return False

def main(path, dimensions=None):
    demo_file_name = path
    demo_file = h5py.File(demo_file_name, "r")


    keys = list(demo_file["data"].keys())
    keys.sort(key=lambda x: int(x.split("_")[1]))
    print(len(keys))
    print(keys)

    num_videos = len(keys) # total number of demo videos to display
    # if dimensions are not given, we display everything in one page. 
    # Otherwise, fit as many as we can in each page
    page_size = num_videos if dimensions is None else dimensions[0] * dimensions[1]
    num_pages = math.ceil(num_videos / page_size)
    final_images = []


    for page in range(num_pages):
        video_source = {}
        max_len = 0
        for demo in keys[page * page_size : page * page_size + page_size]:
            images = demo_file[f"data/{demo}/obs/rgb"][()]
            ra_img = []
            for idx in range(images.shape[0]):
                ra_img.append(images[idx])
                #ra_img.append(images[idx][::-1, :, ::-1])
            video_source[demo] = ra_img
            if max_len < len(images):
                max_len = len(images)
        for run_idx in video_source.keys():
            for _ in range(max_len + 5 - len(video_source[run_idx])):
                video_source[run_idx].append(np.copy(video_source[run_idx][-1]))
        for i in range(max_len + 4):
            imgs = []
            for run_idx in video_source.keys():
                imgs.append(video_source[run_idx][i])
            imgs = np.stack(imgs, axis=0)
            img_tensor = torch.tensor(imgs)
            nrow = 10 if dimensions is None else dimensions[0]
            grid_img = torchvision.utils.make_grid(img_tensor.permute(0, 3, 1, 2), nrow=nrow)
            print(grid_img.shape) # we might need to pad this
            final_images.append(grid_img.permute(1, 2, 0).detach().numpy()[:, :, 2::-1].astype(np.uint8))
        
            
    if not generate_video_stream(final_images, video_name="./all_sim.mp4"):
        print("Failed to generate video")
    print(run_idx)
        
    demo_file.close()
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", type=str)
    parser.add_argument("--width", type=int, default=0)
    parser.add_argument("--height", type=int, default=0)
    args = parser.parse_args()

    if args.width == 0 or args.height == 0:
        dimensions = None
    else:
        dimensions = (args.width, args.height)

    main(args.path, dimensions)
