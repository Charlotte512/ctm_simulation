from math import exp
import numpy as np
from itertools import product


class Cell:
    def __init__(self, car_num=0, length=200, q_max=1600, f_speed=20, dt=10, jam_density=240, av_lane=False,
                 scale_down=1):
        self.car_num = car_num;
        self.AV_num = 0
        if self.car_num == 0:
            self.AV_ratio = 0
        else:
            self.AV_ratio = self.AV_num / self.car_num

        self.length = length;
        self.f_speed = f_speed;
        self.q_max = q_max * (1 + 0.5 * self.AV_ratio)
        self.dt = dt;
        self.jam_density = jam_density * (1 + 0.5 * self.AV_ratio)
        self.wave_speed = self.q_max / (self.jam_density - self.q_max / self.f_speed)
        self.av_lane = av_lane;
        self.q_max = q_max * (1 + 0.5 * self.AV_ratio)
        self.demand = 0;
        self.supply = 0;
        self.density = self.car_num / self.length
        self.scale_down = scale_down;
        self.left_flow = 0
        self.right_flow = 0;
        self.down_flow = 0
        self.left_prob = 0;
        self.right_prob = 0
        self.down_prob = 0;
        self.left_in = 0
        self.right_in = 0;
        self.up_in = 0
        self.exp_right_flow = 0  # 拟向右换道车辆数
        self.exp_left_flow = 0  # 拟向左换道车辆数

    def update_variable(self, left_num, right_num, down_num):
        """
        :param left_num: 左侧元胞车辆数
        :param right_num: 右侧元胞车辆数
        :param down_num: 下游元胞车辆数
        :return:
        """
        beta1 = -0.4;
        beta2 = 5;
        beta3 = -0.1;
        beta4 = 1
        self.demand = min(self.q_max, 1000 * (self.jam_density - self.density) * self.wave_speed * self.dt / 3600)
        print("demand",self.demand)
        self.supply = min(self.q_max, 1000 * self.density * self.f_speed * self.dt / 3600)
        print("supply",self.supply)
        left_utility = beta1 * (self.car_num - left_num) + beta2
        right_utility = beta1 * (self.car_num - right_num) + beta2
        down_utility = beta3 * (self.car_num - down_num) + beta4 - 0.4 * max(0, 16 - down_num)
        self.left_prob = exp(-left_utility) / (exp(-left_utility) + exp(-right_utility) + exp(-down_utility))
        print("left_prob",self.left_prob)
        self.right_prob = exp(-right_utility) / (exp(-left_utility) + exp(-right_utility) + exp(-down_utility))
        self.down_prob = exp(-down_utility) / (exp(-left_utility) + exp(-right_utility) + exp(-down_utility))
        self.exp_down = self.down_prob * self.car_num
        if self.av_lane == True:
            self.right_prob = 1 - self.AV_ratio
            self.down_prob = self.AV_ratio

    def update_car_num(self, up_out, down_supply, left_up_out, left_supply, right_up_out, right_supply, right_lc,
                       left_lc, right_two_lc, left_two_lc, up_av_ratio, left_av_ratio, down_av_ratio):
        """
    :param up_out: 上游元胞想要出来的车辆数
    :param down_supply: 下游元胞的供给
    :param left_up_out: 左侧上游想要直行的车辆数
    :param left_supply: 左侧元胞供给
    :param right_up_out: 右侧上游想要直行的车辆数
    :param right_supply: 右侧元胞供给
    :param right_lc: 右侧想要变道的车辆数
    :param left_lc: 左侧想要变道的车辆数
    :param right_two_lc: 右侧第二个车道往右侧车道换道的车辆数
    :param left_two_two_lc: 左侧第二个车道往左侧车道换道的车辆数
    :return: 主要为更新车辆数
    """
        # 三个进来的车辆数
        self.up_in = min(self.supply, up_out)
        self.right_in = min(right_lc / (right_lc + left_lc) * (self.supply - self.up_in), right_lc)
        self.left_in = min(left_lc / (right_lc + left_lc) * (self.supply - self.up_in), left_lc)
        # 需要根据有多少车换道、直行到当前cell来确定换道对supply的影响；目的：体现出换道多交通的负效用
        lc_ratio = (self.right_in + self.left_in) / (self.right_in + self.left_in + self.up_in)
        self.supply = self.supply * (1 - lc_ratio * 0.2)  # 举例子

        right_up_in = min(right_supply, right_up_out)
        left_up_in = min(left_supply, left_up_out)

        self.exp_right_flow = self.right_prob * self.car_num  # 本车道拟向右换道车辆数
        self.exp_left_flow = self.left_prob * self.car_num  # 本车道拟向右换道车辆数
        # 三个出去的车辆数
        self.down_flow = min(down_supply, self.down_prob * self.car_num)
        self.right_flow = min(self.exp_right_flow / (self.exp_right_flow + right_two_lc) * (right_supply - right_up_in),
                              self.exp_right_flow)
        self.left_flow = min(self.exp_left_flow / (self.exp_left_flow + left_two_lc) * (left_supply - left_up_in),
                             self.exp_left_flow)

        # 更新该元胞内部车辆数
        self.car_num = self.car_num + self.up_in + self.right_in + self.left_in - self.right_flow - self.down_flow - self.right_flow
        if self.av_lane == False:
            self.av_car_num = (
                                          self.car_num - self.right_flow - self.down_flow - self.right_flow) * self.av_ratio + self.up_in * up_av_ratio + self.right_in * right_av_ratio + self.left_in * left_av_ratio
        else:
            self.av_car_num = (self.car_num - self.down_flow) * self.av_ratio + self.up_in * up_av_ratio + self.right_in
        self.AV_ratio = self.av_car_num / self.car_num
        self.wave_speed = self.q_max / (self.jam_density - self.q_max / self.f_speed)
        self.q_max = self.q_max * (1 + 0.5 * self.AV_ratio)

