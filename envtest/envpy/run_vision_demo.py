#!/usr/bin/env python3

import argparse
import os
import random

import cv2
import numpy as np
from flightgym import VisionEnv_v1
from flightrl.rpg_baselines.torch.envs import vec_env_wrapper as wrapper
from ruamel.yaml import YAML, RoundTripDumper, dump


def configure_random_seed(seed, env=None):
    if env is not None:
        env.seed(seed)
    np.random.seed(seed)

def parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=0, help="Random seed")
    parser.add_argument("--render", type=int, default=1, help="Render with Unity")
    return parser


def main():
    args = parser().parse_args()

    # load configurations
    cfg = YAML().load(
        open(
            os.environ["FLIGHTMARE_PATH"] + "/flightpy/configs/vision/config.yaml", "r"
        )
    )

    if args.render:
        cfg["unity"]["render"] = "yes"


    # os.system(os.environ["FLIGHTMARE_PATH"] + "/flightrender/RPG_Flightmare.x86_64 &")

    cfg["simulation"]["num_envs"] = 2 

    # create training environment
    env = VisionEnv_v1(dump(cfg, Dumper=RoundTripDumper), False)
    env = wrapper.FlightEnvVec(env)

    ep_length = 100

    obs_dim = env.obs_dim
    act_dim = env.act_dim
    num_env = env.num_envs

    env.reset(random=True)

    if args.render:
      env.connectUnity()


    for frame_id in range(ep_length):
      print("Simuation step: {0}".format(frame_id))
      # generate action [-1, 1]
      dummy_actions = np.random.rand(num_env, act_dim) * 2 - np.ones(shape=(num_env, act_dim))

      obs, rew, done, info = env.step(dummy_actions)
      #
      receive_frame_id = env.render(frame_id = frame_id)
      print("sending frame id: ", frame_id, "received frame id: ", receive_frame_id)

      # ======RGB Image=========
      raw_rgb_img =env.getImage(rgb=True) 

      num_img = raw_rgb_img.shape[0] 
      num_col = 2
      num_row = int(num_img / num_col)

      rgb_img_list = []
      for col in range(num_col):
        rgb_img_list.append([])
        for row in range(num_row):
          rgb_img = np.reshape(
              raw_rgb_img[col*num_row + row], (env.img_height, env.img_width, 3))
          rgb_img_list[col] += [rgb_img]
      
      rgb_img_tile = cv2.vconcat([cv2.hconcat(im_list_h) for im_list_h in rgb_img_list])
      cv2.imshow("rgb_img", rgb_img_tile)
      if frame_id < 10:
        cv2.imwrite("./images/img_{0:05d}.png".format(frame_id), rgb_img_tile)
      cv2.waitKey(500)

      # ======Depth Image=========
      raw_depth_images = env.getDepthImage()
      depth_img_list = []
      for col in range(num_col):
        depth_img_list.append([])
        for row in range(num_row):
          depth_img = np.reshape(
              raw_depth_images[col*num_row + row], (env.img_height, env.img_width))
          depth_img_list[col] += [depth_img]
      
      depth_img_tile = cv2.vconcat([cv2.hconcat(im_list_h) for im_list_h in depth_img_list])
      cv2.imshow("depth_img", depth_img_tile)
      cv2.waitKey(500)

    #
    if args.render:
        env.disconnectUnity()


if __name__ == "__main__":
    main()

