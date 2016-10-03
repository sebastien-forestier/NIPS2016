import numpy as np
import time

import matplotlib
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Circle
matplotlib.rcParams['figure.figsize'] = (6.0, 6.0)
from numpy import pi, array, linspace, hstack, zeros, transpose
from matplotlib import animation
from IPython.display import HTML, display, Image, clear_output
from ipywidgets import interact_manual
from numpy.random import random, normal

from explauto import SensorimotorModel
from explauto.sensorimotor_model.non_parametric import NonParametric
from explauto import InterestModel
from explauto.interest_model.discrete_progress import DiscretizedProgress
from explauto.utils import rand_bounds, bounds_min_max, softmax_choice, prop_choice
from explauto.environment.dynamic_environment import DynamicEnvironment
from explauto.interest_model.competences import competence_exp, competence_dist
from explauto.environment.modular_environment import FlatEnvironment, HierarchicalEnvironment

from environment import Arm, Ball, Stick, ArmBall, ArmStickBalls
from learning_module import LearningModule
#from utils import compute_explo, display_movement

grid_size = 10


def compute_explo(data, mins, maxs, gs=100):
    n = len(mins)
    if len(data) == 0:
        return 0
    else:
        assert len(data[0]) == n
        epss = (maxs - mins) / gs
        grid = np.zeros([gs] * n)
        for i in range(len(data)):
            idxs = np.array((data[i] - mins) / epss, dtype=int)
            idxs[idxs>=gs] = gs-1
            idxs[idxs<0] = 0
            grid[tuple(idxs)] = grid[tuple(idxs)] + 1
        grid[grid > 1] = 1
        return np.sum(grid)

def display_movement(fig, ax, environment, time_step=0.04):
    fig.show()
    fig.canvas.draw()
    ax.set_aspect('equal')
    ax.set_xlim((-1.5, 1.5))
    ax.set_ylim((-1.5, 1.5))
    background = fig.canvas.copy_from_bbox(ax.bbox)

    for i in range(50):
        start = time.time()
        fig.canvas.restore_region(background)
        lines = environment.env.plot_update(ax, i)
        for line in lines:
            ax.draw_artist(line)
        fig.canvas.blit(ax.bbox)

        end = time.time()
        remain = start + time_step - end
        if remain > 0:
            time.sleep(remain)
    time.sleep(1)
    
    
def random_motor_babbling(trial, iterations):
    env = ArmStickBalls()
    np.random.seed(trial)
    explored_s = []
    res = []
    for iteration in range(iterations):
        m = env.random_motor()
        s = env.update(m)
        if (len(explored_s) == 0) or abs(s[17] - 0.6) > 0.001:
            explored_s += [s]
        if (iteration+1) % (iterations/10) == 0:
            res += [int(compute_explo(array(explored_s)[:,[14,17]], array([-2., -2.]), array([2., 2.]), gs=grid_size))]
    return res

def random_goal_babbling(trial, iterations):
    env = ArmStickBalls()
    np.random.seed(trial)
    explored_s = []
    res = []
    sigma_explo_ratio = 0.05
    sm_model = SensorimotorModel.from_configuration(env.conf, 'nearest_neighbor', 'default')
    m = env.random_motor()
    s = env.update(m)
    sm_model.update(m, s)
    for iteration in range(iterations):
        if (not sm_model.bootstrapped_s) or random() < 0.2:
            m = env.random_motor()
        else:
            s_goal = rand_bounds(env.conf.s_bounds)[0]
            m = sm_model.model.infer_order(tuple(s_goal))
            m = normal(m, sigma_explo_ratio)
        s = env.update(m) # observe the sensory effect s (36D): the trajectory of all objects
        sm_model.update(m, s) # update sensorimotor model
        if (len(explored_s) == 0) or abs(s[17] - 0.6) > 0.001:
            explored_s += [s]
        if (iteration+1) % (iterations/10) == 0:
            res += [int(compute_explo(array(explored_s)[:,[14,17]], array([-2., -2.]), array([2., 2.]), gs=grid_size))]
    return res

def active_model_babbling(trial, iterations):
    env = ArmStickBalls()
    np.random.seed(trial)
    explored_s = []
    res = []
    n_explore=4
    m_ndims = env.conf.m_ndims # number of motor parameters
    m_space = range(m_ndims)
    s_hand  = range(m_ndims, m_ndims+6)
    s_tool  = range(m_ndims+6, m_ndims+12)
    s_ball1 = range(m_ndims+12, m_ndims+18)
    s_ball2 = range(m_ndims+18, m_ndims+24)
    s_ball3 = range(m_ndims+24, m_ndims+30)
    s_ball4 = range(m_ndims+30, m_ndims+36)
    learning_modules = {}
    learning_modules['mod1'] = LearningModule("mod1", m_space, s_hand, env.conf)
    learning_modules['mod2'] = LearningModule("mod2", m_space, s_tool, env.conf)
    learning_modules['mod3'] = LearningModule("mod3", m_space, s_ball1, env.conf)
    learning_modules['mod4'] = LearningModule("mod4", m_space, s_ball2, env.conf)
    learning_modules['mod5'] = LearningModule("mod5", m_space, s_ball3, env.conf)
    learning_modules['mod6'] = LearningModule("mod6", m_space, s_ball4, env.conf)
    for step in range(iterations / (n_explore + 1)):
        interests = [learning_modules[mid].interest() for mid in learning_modules.keys()]
        #interests_evolution.append(interests)
        babbling_module = learning_modules.values()[prop_choice(interests, eps=0.2)]
        m_list = babbling_module.produce(n=n_explore)
        for m in m_list:
            s = env.update(m) # execute this command and observe the corresponding sensory effect
            if (len(explored_s) == 0) or abs(s[17] - 0.6) > 0.001:
                explored_s += [s]
            for mid in learning_modules.keys():
                learning_modules[mid].update_sm(m, learning_modules[mid].get_s(array(list(m) + list(s))))
        m = babbling_module.infer(babbling_module.expl_dims, babbling_module.inf_dims, babbling_module.x, n=1, explore=False)    
        s = env.update(m) # execute this command and observe the corresponding sensory effect
        babbling_module.update_im(m, babbling_module.get_s(array(list(m)+list(s))))
        for mid in learning_modules.keys():
            learning_modules[mid].update_sm(m, learning_modules[mid].get_s(array(list(m) + list(s))))
        if (step+1) % ((iterations / (n_explore + 1))/10) == 0:
            res += [int(compute_explo(array(explored_s)[:,[14,17]], array([-2., -2.]), array([2., 2.]), gs=grid_size))]
    return res

#from multiprocessing import Pool
#from subprocess import call
#import cPickle
#import numpy as np

#trials = 30
#iterations = 100000
    
#def f(condition, trial):
#    call("python run.py " + condition + " " + str(trial) + " " + str(iterations), shell=True)
#    log_dir = './logs/'
#    filename = condition + str(trial) + '.pickle'
#    with open(log_dir + filename, 'r') as f:
#        res = cPickle.load(f)
#    return res

#def run_rmb(trial): return f("rmb", trial)
#def run_rgb(trial): return f("rgb", trial)
#def run_amb(trial): return f("amb", trial)


#if __name__ == '__main__':
#    pool = Pool(30)
#    res_rmb = np.array(pool.map(run_rmb, range(trials)))
#    res_rgb = np.array(pool.map(run_rgb, range(trials)))
#    res_amb = np.array(pool.map(run_amb, range(trials)))

#%matplotlib inline
#fig, ax = plt.subplots()
#x = np.linspace(0, iterations, 11)
#plt.errorbar(x, np.append([0], np.mean(res_amb, axis=0)), np.append([0], np.std(res_amb, axis=0)), lw=2, label="Active Model Babbling")
#plt.errorbar(x, np.append([0], np.mean(res_rgb, axis=0)), np.append([0], np.std(res_rgb, axis=0)), lw=2, label="Random Goal Babbling")
#plt.errorbar(x, np.append([0], np.mean(res_rmb, axis=0)), np.append([0], np.std(res_rmb, axis=0)), lw=2, label="Random Motor Babbling")
#ax.legend(loc="upper left")
#plt.savefig('exploration_stats')