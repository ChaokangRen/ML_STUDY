import random
# reward = [0, 0, 0, 0, 0]

# reward_pre = reward

# ap = 1
# i = 0
# while (True):
#     reward[0] = max(ap * (-1 + reward_pre[0]), ap * (0 + reward_pre[1]))
#     reward[1] = max(ap * (-1 + reward_pre[0]), ap * (-2 + reward_pre[2]))
#     reward[2] = max(ap * (-2 + reward_pre[3]), ap * (0 + reward_pre[4]))
#     reward[3] = max(ap * (10 + reward_pre[4]), ap * (1 + 0.2 *
#                                                      reward_pre[1] + 0.4 * reward_pre[2] + 0.4 * reward_pre[3]))
#     reward[4] = 0
#     reward_pre = reward
#     print(reward)
#     if(i > 5):
#         break
#     i = i + 1

total = [10, 100100000]  # 随机点数
for t in total:
    in_count = 0
    for i in range(t):
        x = random.random()
        y = random.random()
        dis = (x*x + y*y)
        if dis <= 1:
            in_count += 1
    print(t, '个随机点时，π是：', 4*in_count/t)
