function noise = rnd_noise(std_pos, std_vel)

noise = [normrnd(0, std_pos, 2, 1); normrnd(0, std_vel, 2, 1)];
