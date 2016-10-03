import numpy as np
import matplotlib
import matplotlib.pyplot as plt

from matplotlib.patches import Circle


from explauto.environment.dynamic_environment import DynamicEnvironment
from explauto.environment.modular_environment import FlatEnvironment, HierarchicalEnvironment

from explauto.utils import bounds_min_max
from explauto.environment.environment import Environment
from explauto.environment.simple_arm.simple_arm import joint_positions
from explauto.utils.utils import rand_bounds


def arm_lengths(n_joints):
    if n_joints == 3:
        return [0.5, 0.3, 0.2]
    elif n_joints == 7:
        return [0.3, 0.2, 0.2, 0.1, 0.1, 0.05, 0.05]
    else:
        return [1./n_joints] * n_joints
    

class Arm(Environment):
    use_process = True
    def __init__(self, m_mins, m_maxs, s_mins, s_maxs,
                 lengths, angle_shift, rest_state):
        
        Environment.__init__(self, m_mins, m_maxs, s_mins, s_maxs)

        self.lengths = lengths
        self.angle_shift = angle_shift
        self.rest_state = rest_state
        self.reset()
        
    def reset(self):
        #print "reset gripper"
        self.logs = []
        self.lines = None
        
    def compute_motor_command(self, m):
        return bounds_min_max(m, self.conf.m_mins, self.conf.m_maxs)
        #return m

    def compute_sensori_effect(self, m):
        a = self.angle_shift + np.cumsum(np.array(m))
        a_pi = np.pi * a 
        hand_pos = np.array([np.sum(np.cos(a_pi)*self.lengths), np.sum(np.sin(a_pi)*self.lengths)])
        angle = np.mod(a[-1] + 1, 2) - 1
        self.logs.append(m)
        return [hand_pos[0], hand_pos[1], angle]    
        
    def plot(self, ax, i, **kwargs_plot):
        m = self.logs[i]
        angles = np.array(m)
        angles[0] += self.angle_shift
        x, y = joint_positions(angles, self.lengths, 'std')
        x, y = [np.hstack((0., a)) for a in x, y]
        l = []
        l += ax.plot(x, y, 'grey', lw=4, animated=True, **kwargs_plot)
        l += ax.plot(x[0], y[0], 'sk', ms=8, animated=True, **kwargs_plot)
        for i in range(len(self.lengths)-1):
            l += ax.plot(x[i+1], y[i+1], 'ok', ms=8, animated=True, **kwargs_plot)
        l += ax.plot(x[-1], y[-1], 'or', ms=8, animated=True, **kwargs_plot)
        self.lines = l
        return l 
        
    def plot_update(self, ax, i, **kwargs_plot):
        if self.lines is None:
            self.plot(ax, 0, **kwargs_plot)
        m = self.logs[i]
        angles = np.array(m)
        angles[0] += self.angle_shift
        x, y = joint_positions(angles, self.lengths, 'std')
        x, y = [np.hstack((0., a)) for a in x, y]
        l = []
        l += [[x, y]]
        l += [[x[0], y[0]]]
        for i in range(len(self.lengths)-1):
            l += [[x[i+1], y[i+1]]]
        l += [[x[-1], y[-1]]]
        for (line, data) in zip(self.lines, l):
            line.set_data(data[0], data[1])
        return self.lines

        
class Ball(Environment):
    def __init__(self, m_mins, m_maxs, s_mins, s_maxs,
                 size, initial_position, ball_type="magnetic", color='y', random_ball_noise=0.2):
        
        Environment.__init__(self, m_mins, m_maxs, s_mins, s_maxs)

        self.ball_type = ball_type
        self.size = size
        self.size_sq = size * size
        self.color = color
        self.initial_position = initial_position
        self.random_ball_noise = random_ball_noise
        self.reset()
        
        
    def reset(self):
        self.move = False
        self.circle = None
        self.pos = np.array(self.initial_position)
        self.logs = []
        
    def compute_motor_command(self, m):
        return m

    def compute_sensori_effect(self, m):
        if self.ball_type == "magnetic":
            if self.move or ((m[0] - self.pos[0]) ** 2 + (m[1] - self.pos[1]) ** 2 < self.size_sq):
                self.pos = m[0:2]
                self.move = 1
        elif self.ball_type == "random":
            self.pos = self.pos + np.random.randn(2) * self.random_ball_noise
             
        self.logs.append([self.pos,
                          self.move])
        return list(self.pos)
    
    def plot(self, ax, i, **kwargs_plot):
        self.logs = self.logs[-50:]
        pos = self.logs[i][0]     
        self.circle = Circle((pos[0], pos[1]), self.size, fc=self.color, animated=True, **kwargs_plot)
        ax.add_patch(self.circle)  
        return [self.circle]
        
    def plot_update(self, ax, i, **kwargs_plot):
        if self.circle is None:
            self.plot(ax, 0, **kwargs_plot)
        self.logs = self.logs[-50:]
        pos = self.logs[i][0]    
        self.circle.center = tuple(pos)
        return [self.circle]
        
        
class ArmBall(DynamicEnvironment):
    def __init__(self, n_joints=3, n_dmp_basis=3, goal_size=1.):
        
        arm_config = dict(
            m_mins=[-1.] * n_joints,
            m_maxs=[1.] * n_joints, 
            s_mins=[-1.] * 3,
            s_maxs=[1.] * 3, 
            lengths=arm_lengths(n_joints), 
            angle_shift=0.5,
            rest_state=[0.] * n_joints)
        
        ball_config = dict(
            m_mins=[-1.] * 2,
            m_maxs=[1.] * 2, 
            s_mins=[-1.] * 2,
            s_maxs=[1.] * 2,
            size=0.1,
            initial_position=[0.6, 0.6],
            color="b")
        
        arm_ball_cfg = dict(
            m_mins=[-1.] * n_joints,
            m_maxs=[1.] * n_joints,
            s_mins=[-1.] * 2,
            s_maxs=[1.] * 2,
            top_env_cls=Ball, 
            lower_env_cls=Arm, 
            top_env_cfg=ball_config, 
            lower_env_cfg=arm_config, 
            fun_m_lower= lambda m:m,
            fun_s_lower=lambda m,s:s[0:2],
            fun_s_top=lambda m,s_lower,s:s)
        
        dynamic_environment_config = dict(
            env_cfg=arm_ball_cfg,
            env_cls=HierarchicalEnvironment,
            m_mins=[-1.] * n_joints * n_dmp_basis, 
            m_maxs=[1.] * n_joints * n_dmp_basis, 
            s_mins=[-goal_size] * 2,
            s_maxs=[goal_size] * 2,
            n_bfs=n_dmp_basis,
            move_steps=50, 
            n_dynamic_motor_dims=n_joints,
            n_dynamic_sensori_dims=2,
            sensori_traj_type="end_point",
            max_params=1000)
        
        DynamicEnvironment.__init__(self, **dynamic_environment_config)
        
    def random_motor(self): return self.random_motors()[0]
        
       
class Stick(Environment):
    def __init__(self, m_mins, m_maxs, s_mins, s_maxs,
                 length, handle_tol, rest_state):
        
        Environment.__init__(self, m_mins, m_maxs, s_mins, s_maxs)

        self.length = length
        self.handle_tol = handle_tol
        self.handle_tol_sq = handle_tol * handle_tol
        self.rest_state = rest_state
        
        self.reset()


    def reset(self):
        self.lines = None
        self.held = False
        self.handle_pos = np.array(self.rest_state[0:2])
        self.angle = self.rest_state[2]
        self.compute_end_pos()
        self.logs = []
        
    def compute_end_pos(self):
        a = np.pi * self.angle
        self.end_pos = [self.handle_pos[0] + np.cos(a) * self.length, 
                        self.handle_pos[1] + np.sin(a) * self.length]
                
    def compute_motor_command(self, m):
        return m

    def compute_sensori_effect(self, m):
        hand_pos = m[0:2]
        hand_angle = m[2]
        
        if not self.held:
            if (hand_pos[0] - self.handle_pos[0]) ** 2. + (hand_pos[1] - self.handle_pos[1]) ** 2. < self.handle_tol_sq:
                self.handle_pos = hand_pos
                self.angle = hand_angle
                #self.angle = np.mod(hand_angle + self.handle_noise * np.random.randn() + 1, 2) - 1
                self.compute_end_pos()
                self.held = True
        else:
            self.handle_pos = hand_pos
            self.angle = hand_angle
            #self.angle = np.mod(hand_angle + self.handle_noise * np.random.randn() + 1, 2) - 1
            self.compute_end_pos()
        
        self.logs.append([self.handle_pos, 
                          self.angle, 
                          self.end_pos, 
                          self.held])
        #print "Tool hand_pos:", hand_pos, "hand_angle:", hand_angle, "gripper_change:", gripper_change, "self.handle_pos:", self.handle_pos, "self.angle:", self.angle, "self.held:", self.held 
        return list(self.end_pos) # Tool pos
    
    def plot(self, ax, i, **kwargs_plot):
        handle_pos = self.logs[i][0]
        end_pos = self.logs[i][2]
        l = []
        l += ax.plot([handle_pos[0], end_pos[0]], [handle_pos[1], end_pos[1]], '-', color="g", lw=6, animated=True, **kwargs_plot)
        l += ax.plot(handle_pos[0], handle_pos[1], 'o', color = "r", ms=12, animated=True, **kwargs_plot)
        l += ax.plot(end_pos[0], end_pos[1], 'o', color = "b", ms=12, animated=True, **kwargs_plot)
        self.lines = l
        return l 
        
    def plot_update(self, ax, i, **kwargs_plot):
        if self.lines is None:
            self.plot(ax, 0, **kwargs_plot)
        handle_pos = self.logs[i][0]
        end_pos = self.logs[i][2]
        
        l = [[[handle_pos[0], end_pos[0]], [handle_pos[1], end_pos[1]]]]
        l += [[handle_pos[0], handle_pos[1]]]
        l += [[end_pos[0], end_pos[1]]]
        for (line, data) in zip(self.lines, l):
            line.set_data(data[0], data[1])
        return self.lines

        
class ArmStickBalls(DynamicEnvironment):
    def __init__(self, 
                 n_joints=7, 
                 n_dmp_basis=3, 
                 goal_size=2., 
                 stick_handle_tol=0.05, 
                 stick_length=0.3, 
                 ball_size=0.1,
                 random_ball_noise=0.2):

        arm_config = dict(
            m_mins=[-1.] * n_joints,
            m_maxs=[1.] * n_joints, 
            s_mins=[-1.] * 3,
            s_maxs=[1.] * 3, 
            lengths=arm_lengths(n_joints), 
            angle_shift=0.5,
            rest_state=[0.] * n_joints)
        
        stick_config = dict(
            m_mins=[-1, -1, -1], # Hand pos + arm angle
            m_maxs=[1, 1, 1], 
            s_mins=[-2, -2],  # Tool pos
            s_maxs=[2, 2],
            length=stick_length, 
            handle_tol=stick_handle_tol, 
            rest_state=[-0.75, 0.25, 0.75])
                
        arm_stick_cfg = dict(
            m_mins=list([-1.] * n_joints), # 3DOF + gripper
            m_maxs=list([1.] * n_joints),
            s_mins=list([-1.] * 4),
            s_maxs=list([1.] * 4),
            top_env_cls=Stick, 
            lower_env_cls=Arm, 
            top_env_cfg=stick_config, 
            lower_env_cfg=arm_config, 
            fun_m_lower= lambda m:m,
            fun_s_lower=lambda m,s:s,
            fun_s_top=lambda m,s_lower,s:s_lower[0:2] + s) # from s: Tool end pos  from m: hand_pos
                
        ball_config = dict(
            m_mins=[-2.] * 2,
            m_maxs=[2.] * 2, 
            s_mins=[-2.] * 2,
            s_maxs=[2.] * 2,
            size=ball_size,
            initial_position=[0.6, 0.6],
            ball_type="magnetic",
            color="b")
        
        static_ball_config = dict(
            m_mins=[-2.] * 2,
            m_maxs=[2.] * 2, 
            s_mins=[-2.] * 2,
            s_maxs=[2.] * 2,
            size=0.1,
            initial_position=[-0.6, 0.6],
            ball_type="static",
            color="m")
        
        random_ball_config = dict(
            m_mins=[-2.] * 2,
            m_maxs=[2.] * 2, 
            s_mins=[-2.] * 2,
            s_maxs=[2.] * 2,
            size=0.1,
            initial_position=[0.3, 1.],
            ball_type="random",
            color="y",
            random_ball_noise=random_ball_noise)
        
        random_ball2_config = dict(
            m_mins=[-2.] * 2,
            m_maxs=[2.] * 2, 
            s_mins=[-2.] * 2,
            s_maxs=[2.] * 2,
            size=0.1,
            initial_position=[0.3, 1.],
            ball_type="random",
            color="g",
            random_ball_noise=random_ball_noise)
        
        balls_config = dict(
            s_mins = list([-2.]*8),
            s_maxs = list([2.]*8),
            envs_cls = [Ball, Ball, Ball, Ball], 
            envs_cfg = [ball_config, static_ball_config, random_ball_config, random_ball2_config], 
            combined_s = lambda s:s)
        
        arm_stick_balls_cfg = dict(
            m_mins=[-1.] * n_joints,
            m_maxs=[1.] * n_joints,
            s_mins=[-1.] * 12,
            s_maxs=[1.] * 12,
            top_env_cls=FlatEnvironment, 
            lower_env_cls=HierarchicalEnvironment, 
            top_env_cfg=balls_config, 
            lower_env_cfg=arm_stick_cfg, 
            fun_m_lower= lambda m:m,
            fun_s_lower=lambda m,s:s[2:4]+s[2:4]+s[2:4]+s[2:4],
            fun_s_top=lambda m,s_lower,s:s_lower + s)
            
        dynamic_environment_config = dict(
            env_cfg=arm_stick_balls_cfg,
            env_cls=HierarchicalEnvironment,
            m_mins=[-1.] * n_dmp_basis * n_joints, 
            m_maxs=[1.] * n_dmp_basis * n_joints, 
            s_mins=[-goal_size] * n_dmp_basis * 12,
            s_maxs=[goal_size] * n_dmp_basis * 12,
            n_bfs=n_dmp_basis,
            move_steps=50, 
            n_dynamic_motor_dims=n_joints,
            n_dynamic_sensori_dims=8, 
            sensori_traj_type="samples",
            max_params=1000)

        DynamicEnvironment.__init__(self, **dynamic_environment_config)

    def random_motor(self): return self.random_motors()[0]
    