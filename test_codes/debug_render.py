import gym

if __name__ == '__main__':
    env = gym.make('Walker2d-v2')
    env.reset()
    env.render()
    imgs = []
    fps = env.metadata.get('video.frames_per_second')
    dt = env.dt
    print(fps, dt)
    for i in range(2000):
        env.step(env.action_space.sample())
        # env.env.viewer.vopt.flags[mujoco_py.const.VIS_CONTACTFORCE] = 1
        env.render()
