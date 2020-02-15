#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math

class NMPCSimulatorSystem():
    """SimulatorSystem for nmpc, this is the simulator of nmpc
    the reason why I seperate the real simulator and nmpc's simulator is sometimes the modeling error, disturbance can include in real simulator
    Attributes
    -----------
    None

    """
    def __init__(self):
        """
        Parameters
        -----------
        None
        """
        self.state_ref = []
        self.w = 1
        self.x11 = 3.5
        self.x22 = 6.5
        self.y11 = 3
        self.y22 = 7
        self.sigma = 5
        self.delta = 0.15
        self.P1 = 3
        self.P2 = 3
        self.P3 = 4
        self.Q1 = 1
        self.Q2 = 1
        self.Q3 = 2

    def calc_predict_and_adjoint_state(self, x_1, x_2, x_3, u_1s, u_2s, u_3s, N, dt):
        """main
        Parameters
        ------------
        x_1 : float
            current state
        x_2 : float
            current state
        x_3 : float
            current state
        u_1s : list of float
            estimated optimal input Us for N steps
        u_2s : list of float
            estimated optimal input Us for N steps
        u_3s : list of float
            estimated optimal input Us for N steps
        N : int
            predict step
        dt : float
            sampling time

        Returns
        --------
        x_1s : list of float
            predicted x_1s for N steps
        x_2s : list of float
            predicted x_2s for N steps
        x_3s : list of float
            predicted x_3s for N steps
        lam_1s : list of float
            adjoint state of x_1s, lam_1s for N steps
        lam_2s : list of float
            adjoint state of x_2s, lam_2s for N steps
        lam_3s : list of float
            adjoint state of x_3s, lam_3s for N steps
        """
        x_1s, x_2s, x_3s = self._calc_predict_states(x_1, x_2, x_3, u_1s, u_2s, u_3s, N, dt) # by usin state equation
        lam_1s, lam_2s, lam_3s = self._calc_adjoint_states(x_1s, x_2s, x_3s, u_1s, u_2s, u_3s, N, dt) # by using adjoint equation

        return x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s

    def _calc_predict_states(self, x_1, x_2, x_3, u_1s, u_2s, u_3s, N, dt):
        """ 
        Parameters
        ------------
        x_1 : float
            current state
        x_2 : float
            current state
        x_3 : float
            current state
        u_1s : list of float
            estimated optimal input Us for N steps
        u_2s : list of float
            estimated optimal input Us for N steps
        u_3s : list of float
            estimated optimal input Us for N steps
        N : int
            predict step
        dt : float
            sampling time
            
        Returns
        --------
        x_1s : list of float
            predicted x_1s for N steps
        x_2s : list of float
            predicted x_2s for N steps
        x_3s : list of float
            predicted x_3s for N steps
        """
        # initial state
        x_1s = [x_1]
        x_2s = [x_2]
        x_3s = [x_3]

        for i in range(N):
            temp_x_1, temp_x_2, temp_x_3 = self._predict_state_with_oylar(x_1s[i], x_2s[i], x_3s[i], u_1s[i], u_2s[i], u_3s[i], dt)
            x_1s.append(temp_x_1)
            x_2s.append(temp_x_2)
            x_3s.append(temp_x_3)

        return x_1s, x_2s, x_3s

    def _calc_adjoint_states(self, x_1s, x_2s, x_3s, u_1s, u_2s, u_3s, N, dt):
        """
        Parameters
        ------------
        x_1s : list of float
            predicted x_1s for N steps
        x_2s : list of float
            predicted x_2s for N steps
        x_3s : list of float
            predicted x_3s for N steps
        u_1s : list of float
            estimated optimal input Us for N steps
        u_2s : list of float
            estimated optimal input Us for N steps
        u_3s : list of float
            estimated optimal input Us for N steps
        N : int
            predict step
        dt : float
            sampling time
            
        Returns
        --------
        lam_1s : list of float
            adjoint state of x_1s, lam_1s for N steps
        lam_2s : list of float
            adjoint state of x_2s, lam_2s for N steps
        lam_3s : list of float
            adjoint state of x_2s, lam_2s for N steps
        """
        # final state
        # final_state_func
       
        #終端コスト
        lam_1s = [self.P1*(x_1s[-1]-self.state_ref[0])]
        lam_2s = [self.P2*(x_2s[-1]-self.state_ref[1])]
        lam_3s = [self.P3*(x_3s[-1]-self.state_ref[2])]

        for i in range(N-1, 0, -1): 
            temp_lam_1, temp_lam_2, temp_lam_3 = self._adjoint_state_with_oylar(x_1s[i], x_2s[i], x_3s[i], lam_1s[0] ,lam_2s[0], lam_3s[0], u_1s[i], u_2s[i], u_3s[i], dt)
            lam_1s.insert(0, temp_lam_1)
            lam_2s.insert(0, temp_lam_2)
            lam_3s.insert(0, temp_lam_3)

        return lam_1s, lam_2s, lam_3s

    def final_state_func(self, goals):
        """this func usually need
        """
        self.state_ref = goals

    def _predict_state_with_oylar(self, x_1, x_2, x_3, u_1, u_2, u_3, dt):
        """in this case this function is the same as simulator
        Parameters
        ------------
        x_1 : float
            system state
        x_2 : float
            system state
        x_3 : float
            system state
        u_1 : float
            system input
        u_2 : float
            system input
        u_3 : float
            system input
        dt : float in seconds
            sampling time
        Returns
        --------
        next_x_1 : float
            next state, x_1 calculated by using state equation
        next_x_2 : float
            next state, x_2 calculated by using state equation
        next_x_3 : float
            next state, x_3 calculated by using state equation
        """
        k0 = [0. for _ in range(3)]

        functions = [self.func_x_1, self.func_x_2, self.func_x_3]

        for i, func in enumerate(functions):
            k0[i] = dt * func(x_1, x_2, x_3, u_1, u_2, u_3)
                
        next_x_1 = x_1 + k0[0]
        next_x_2 = x_2 + k0[1]
        next_x_3 = x_3 + k0[2]

        return next_x_1, next_x_2, next_x_3

    def func_x_1(self, y_1, y_2, y_3, u_1, u_2, u_3):
        """
        Parameters
        ------------
        y_1 : float
        y_2 : float
        y_3 : float
        u_1 : float
            system input
        u_2 : float
            system input
        u_3 : float
            system input
        """
        y_dot =  u_1
        return y_dot
    
    def func_x_2(self, y_1, y_2, y_3, u_1, u_2, u_3):
        """
        Parameters
        ------------
        y_1 : float
        y_2 : float
        y_3 : float
        u_1 : float
            system input
        u_2 : float
            system input
        u_3 : float
            system input
        """
        y_dot =  u_2
        return y_dot
    
    def func_x_3(self, y_1, y_2, y_3, u_1, u_2, u_3):
        """
        Parameters
        ------------
        y_1 : float
        y_2 : float
        y_3 : float
        u_1 : float
            system input
        u_2 : float
            system input
        u_3 : float
            system input
        """
        y_dot = u_3
        return y_dot

    def _adjoint_state_with_oylar(self, x_1, x_2, x_3, lam_1, lam_2, lam_3, u_1, u_2, u_3, dt):
        """
        Parameters
        ------------
        x_1 : float
            system state
        x_2 : float
            system state
        x_3 : float
            system state
        lam_1 : float
            adjoint state
        lam_2 : float
            adjoint state
        lam_3 : float
            adjoint state
        u_1 : float
            system input
        u_2 : float
            system input
        u_3 : float
            system input
        dt : float in seconds
            sampling time
        Returns
        --------
        pre_lam_1 : float
            pre, 1 step before lam_1 calculated by using adjoint equation
        pre_lam_2 : float
            pre, 1 step before lam_2 calculated by using adjoint equation
        pre_lam_3 : float
            pre, 1 step before lam_3 calculated by using adjoint equation
        """
        k0 = [0. for _ in range(3)]

        functions = [self._func_lam_1, self._func_lam_2, self._func_lam_3]

        for i, func in enumerate(functions):
            k0[i] = dt * func(x_1, x_2, x_3, lam_1, lam_2, lam_3, u_1, u_2, u_3)
                
        pre_lam_1 = lam_1 + k0[0]
        pre_lam_2 = lam_2 + k0[1]
        pre_lam_3 = lam_3 + k0[2]

        return pre_lam_1, pre_lam_2, pre_lam_3

    def _func_lam_1(self, y_1, y_2, y_3, y_4, y_5, y_6, u_1, u_2, u_3):
        """calculating -\dot{lam_1}
        Parameters
        ------------
        y_1 : float
            means x_1
        y_2 : float
            means x_2
        y_3 : float
            means x_3
        y_4 : float
            means lam_1
        y_5 : float
            means lam_2
        y_6 : float
            means lam_3
        u_1 : float
            means system input
        u_2 : float
            means system input
        u_3 : float
            means system input
        Returns
        ---------
        y_dot : float
            means -\dot{lam_1}
        """
        '''
        y_dot = float()
        if(y_1<self.delta):
            y_dot += self.w * (y_1 + self.delta)
        elif(y_1>10-self.delta):
            y_dot += self.w * (y_1 - 10 + self.delta)

        if(y_2<self.y22):
            y_dot += self.w * math.exp(-(y_1-self.x22)**2/(2 * self.sigma**2))*(-(y_1-self.x22)/self.sigma**2)
        elif(y_2>self.y11):
            y_dot += self.w * math.exp(-(y_1-self.x11)**2/(2 * self.sigma**2))*(-(y_1-self.x11)/self.sigma**2)
        '''

        y_dot = self.Q1*(y_1 - self.state_ref[0])
        return y_dot

    def _func_lam_2(self, y_1, y_2, y_3, y_4, y_5, y_6, u_1, u_2, u_3):
        """calculating -\dot{lam_2}
        Parameters
        ------------
        y_1 : float
            means x_1
        y_2 : float
            means x_2
        y_3 : float
            means x_3
        y_4 : float
            means lam_1
        y_5 : float
            means lam_2
        y_6 : float
            means lam_3
        u_1 : float
            means system input
        u_2 : float
            means system input
        u_3 : float
            means system input
        Returns
        ---------
        y_dot : float
            means -\dot{lam_2}
        """
        '''
        y_dot = float()
        if(y_2<self.delta):
            y_dot += self.w * (y_2 + self.delta)
        elif(y_2>10-self.delta):
            y_dot += self.w * (y_2 - 10 + self.delta)
        '''
        y_dot = self.Q2*(y_2 - self.state_ref[1])
        return y_dot

    def _func_lam_3(self, y_1, y_2, y_3, y_4, y_5, y_6, u_1, u_2, u_3):
        """calculating -\dot{lam_3}
        Parameters
        ------------
        y_1 : float
            means x_1
        y_2 : float
            means x_2
        y_3 : float
            means x_3
        y_4 : float
            means lam_1
        y_5 : float
            means lam_2
        y_6 : float
            means lam_3
        u_1 : float
            means system input
        u_2 : float
            means system input
        u_3 : float
            means system input
        Returns
        ---------
        y_dot : float
            means -\dot{lam_3}
        """
        y_dot = self.Q3*(y_3 - self.state_ref[2]) 
        return y_dot

class NMPCController_with_CGMRES():
    """
    Attributes
    ------------
    zeta : float
        gain of optimal answer stability
    ht : float
        update value of NMPC this should be decided by zeta
    tf : float
        predict time
    alpha : float
        gain of predict time
    N : int
        predicte step, discritize value
    threshold : float
        cgmres's threshold value
    input_num : int
        system input length, this should include dummy u and constraint variables
    max_iteration : int
        decide by the solved matrix size
    simulator : NMPCSimulatorSystem class
    u_1s : list of float
        estimated optimal system input
    u_2s : list of float
        estimated optimal system input
    u_3s : list of float
        estimated optimal system input
    dummy_u_1s : list of float
        estimated dummy input
    dummy_u_2s : list of float
        estimated dummy input
    raw_1s : list of float
        estimated constraint variable
    raw_2s : list of float
        estimated constraint variable
    history_u_1 : list of float
        time history of actual system input
    history_u_2 : list of float
        time history of actual system input
    history_u_3 : list of float
        time history of actual system input
    history_dummy_u_1 : list of float
        time history of actual dummy u_1
    history_dummy_u_2 : list of float
        time history of actual dummy u_2
    history_raw_1 : list of float
        time history of actual raw_1
    history_raw_2 : list of float
        time history of actual raw_2
    history_f : list of float
        time history of error of optimal
    """
    def __init__(self):
        """
        Parameters
        -----------
        None
        """
        # parameters
        self.zeta = 100. # 安定化ゲイン
        self.ht = 0.01 # 差分近似の幅
        self.tf = 1 # 最終時間
        self.alpha = 0.5 # 時間の上昇ゲイン
        self.N = 10 # 分割数
        self.threshold = 0.001 # break値

        self.input_num = 7 # dummy, 制約条件に対するuにも合わせた入力の数
        self.max_iteration = self.input_num * self.N

        # simulator
        self.simulator = NMPCSimulatorSystem()
        self.R1 = 1
        self.R2 = 1
        self.R3 = 1


        # initial
        self.u_1s = np.ones(self.N) 
        self.u_2s = np.ones(self.N) 
        self.u_3s = np.ones(self.N) 
        self.dummy_u_1s = np.ones(self.N) 
        self.dummy_u_2s = np.ones(self.N) 
        self.raw_1s = np.ones(self.N) 
        self.raw_2s = np.ones(self.N) 

        self.pre_cmd_pose = []

        # for fig
        self.history_u_1 = []
        self.history_u_2 = []
        self.history_u_3 = []
        self.history_dummy_u_1 = []
        self.history_dummy_u_2 = []
        self.history_raw_1 = []
        self.history_raw_2 = []
        self.history_f = []

    def calc_input(self, x_1, x_2, x_3,cmd_pose, time):
        """
        Parameters
        ------------
        x_1 : float
            current state
        x_2 : float
            current state
        x_3 : float
            current state
        time : float in seconds
            now time
        Returns
        --------
        u_1s : list of float
            estimated optimal system input
        u_2s : list of float
            estimated optimal system input
        u_3s : list of float
            estimated optimal system input
        """
        # calculating sampling time
        
        if(cmd_pose != self.pre_cmd_pose):

            self.u_1s = np.ones(self.N) *0#* (-cmd_pose[0]/(cmd_pose[0]+cmd_pose[1] + 10e2))
            self.u_2s = np.ones(self.N) *0#* (-cmd_pose[1]/(cmd_pose[0]+cmd_pose[1]+ + 10e2))
            self.u_3s = np.ones(self.N) *0#* (-1.5)
            self.dummy_u_1s = np.ones(self.N) #* 0.01/(cmd_pose[0]+cmd_pose[1]+ 10e2)
            self.dummy_u_2s = np.ones(self.N) /6.6#* 0.03/(2*cmd_pose[2]+ 10e2)
            self.raw_1s = np.ones(self.N) /200#* (cmd_pose[0]+cmd_pose[1])/2
            self.raw_2s = np.ones(self.N) *0.033#* cmd_pose[2]/3

        self.pre_cmd_pose = cmd_pose
        self.simulator.final_state_func(cmd_pose)
       
        dt = self.tf * (1. - np.exp(-self.alpha * time)) / float(self.N)

        # x_dot
        x_1_dot = self.simulator.func_x_1(x_1, x_2,  x_3, self.u_1s[0], self.u_2s[0], self.u_3s[0])
        x_2_dot = self.simulator.func_x_2(x_1, x_2, x_3, self.u_1s[0], self.u_2s[0], self.u_3s[0])
        x_3_dot = self.simulator.func_x_3(x_1, x_2, x_3, self.u_1s[0], self.u_2s[0], self.u_3s[0])

        dx_1 = x_1_dot * self.ht
        dx_2 = x_2_dot * self.ht
        dx_3 = x_3_dot * self.ht

        x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(x_1 + dx_1, x_2 + dx_2,  x_3 + dx_3, self.u_1s, self.u_2s, self.u_3s, self.N, dt)
        
        # Fxt
        Fxt = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s, self.u_1s, self.u_2s, self.u_3s, self.dummy_u_1s,  self.dummy_u_2s,
                            self.raw_1s, self.raw_2s, self.N, dt)

        # F
        x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(x_1, x_2, x_3, self.u_1s, self.u_2s, self.u_3s, self.N, dt)

        F = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s, self.u_1s, self.u_2s, self.u_3s, self.dummy_u_1s,  self.dummy_u_2s,
                            self.raw_1s, self.raw_2s, self.N, dt)

        right = -self.zeta * F - ((Fxt - F) / self.ht)

        du_1 = self.u_1s * self.ht
        du_2 = self.u_2s * self.ht
        du_3 = self.u_3s * self.ht
        ddummy_u_1 = self.dummy_u_1s * self.ht
        ddummy_u_2 = self.dummy_u_2s * self.ht
        draw_1 = self.raw_1s * self.ht
        draw_2 = self.raw_2s * self.ht

        x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(x_1 + dx_1, x_2 + dx_2, x_3 + dx_3, self.u_1s + du_1, self.u_2s + du_2, self.u_3s + du_3, self.N, dt)

        Fuxt = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s, self.u_1s + du_1, self.u_2s + du_2, self.u_3s + du_3, self.dummy_u_1s + ddummy_u_1,  self.dummy_u_2s + ddummy_u_2, 
                           self.raw_1s + draw_1, self.raw_2s + draw_2, self.N, dt)

        left = ((Fuxt - Fxt) / self.ht)

        # calculationg cgmres
        r0 = right - left
        r0_norm = np.linalg.norm(r0)
        
        vs = np.zeros((self.max_iteration, self.max_iteration + 1)) # 数×iterarion回数
        
        vs[:, 0] = r0 / r0_norm # 最初の基底を算出

        hs = np.zeros((self.max_iteration + 1, self.max_iteration + 1))

        e = np.zeros((self.max_iteration + 1, 1)) # in this case the state is 3(u and dummy_u)
        e[0] = 1.

        for i in range(self.max_iteration):
            du_1 = vs[::self.input_num, i] * self.ht
            du_2 = vs[1::self.input_num, i] * self.ht
            du_3 = vs[2::self.input_num, i] * self.ht
            ddummy_u_1 = vs[3::self.input_num, i] * self.ht
            ddummy_u_2 = vs[4::self.input_num, i] * self.ht            
            draw_1 = vs[5::self.input_num, i] * self.ht
            draw_2 = vs[6::self.input_num, i] * self.ht

            x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(x_1 + dx_1, x_2 + dx_2, x_3 + dx_3, self.u_1s + du_1, self.u_2s + du_2, self.u_3s + du_3, self.N, dt)

            Fuxt = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s, self.u_1s + du_1, self.u_2s + du_2, self.u_3s + du_3, self.dummy_u_1s + ddummy_u_1,  self.dummy_u_2s + ddummy_u_2, 
                           self.raw_1s + draw_1, self.raw_2s + draw_2, self.N, dt)

            Av = (( Fuxt - Fxt) / self.ht)

            sum_Av = np.zeros(self.max_iteration)

            for j in range(i + 1): # グラムシュミットの直交化法です、和を取って差分を取って算出します
                hs[j, i] = np.dot(Av, vs[:, j])
                sum_Av = sum_Av + hs[j, i] * vs[:, j]

            v_est = Av - sum_Av

            hs[i+1, i] = np.linalg.norm(v_est)

            vs[:, i+1] = v_est / hs[i+1, i]

            inv_hs = np.linalg.pinv(hs[:i+1, :i]) # この辺は教科書（実時間の方）にのっています
            ys = np.dot(inv_hs, r0_norm * e[:i+1])

            judge_value = r0_norm * e[:i+1] - np.dot(hs[:i+1, :i], ys[:i])

            if np.linalg.norm(judge_value) < self.threshold or i == self.max_iteration-1:
                update_value = np.dot(vs[:, :i-1], ys_pre[:i-1]).flatten()
                du_1_new = du_1 + update_value[::self.input_num]
                du_2_new = du_2 + update_value[1::self.input_num]
                du_3_new = du_3 + update_value[2::self.input_num]
                ddummy_u_1_new = ddummy_u_1 + update_value[3::self.input_num]
                ddummy_u_2_new = ddummy_u_2 + update_value[4::self.input_num]
                draw_1_new = draw_1 + update_value[5::self.input_num]
                draw_2_new = draw_2 + update_value[6::self.input_num]
                break

            ys_pre = ys
        
        # update
        self.u_1s += du_1_new * self.ht
        self.u_2s += du_2_new * self.ht
        self.u_3s += du_3_new * self.ht
        self.dummy_u_1s += ddummy_u_1_new * self.ht
        self.dummy_u_2s += ddummy_u_2_new * self.ht
        self.raw_1s += draw_1_new * self.ht
        self.raw_2s += draw_2_new * self.ht

        x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s = self.simulator.calc_predict_and_adjoint_state(x_1, x_2, x_3, self.u_1s, self.u_2s,  self.u_3s,self.N, dt)

        F = self._calc_f(x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s, self.u_1s, self.u_2s,  self.u_3s,self.dummy_u_1s,  self.dummy_u_2s,
                         self.raw_1s, self.raw_2s, self.N, dt)

        #print("check F = {0}".format(np.linalg.norm(F)))

        # for save
        self.history_f.append(np.linalg.norm(F))
        self.history_u_1.append(self.u_1s[0])
        self.history_u_2.append(self.u_2s[0])
        self.history_u_3.append(self.u_3s[0])
        self.history_dummy_u_1.append(self.dummy_u_1s[0])
        self.history_dummy_u_2.append(self.dummy_u_2s[0])
        self.history_raw_1.append(self.raw_1s[0])
        self.history_raw_2.append(self.raw_2s[0])

        return self.u_1s, self.u_2s, self.u_3s, x_1s, x_2s

    def _calc_f(self, x_1s, x_2s, x_3s, lam_1s, lam_2s, lam_3s, u_1s, u_2s,  u_3s,dummy_u_1s, dummy_u_2s, raw_1s, raw_2s, N, dt):
        """
        Parameters
        ------------
        x_1s : list of float
            predicted x_1s for N steps
        x_2s : list of float
            predicted x_2s for N steps
        x_3s : list of float
            predicted x_3s for N steps
        lam_1s : list of float
            adjoint state of x_1s, lam_1s for N steps
        lam_2s : list of float
            adjoint state of x_2s, lam_2s for N steps
        lam_3s : list of float
            adjoint state of x_2s, lam_3s for N steps
        u_1s : list of float
            estimated optimal system input
        u_2s : list of float
            estimated optimal system input
        u_3s : list of float
            estimated optimal system input
        dummy_u_1s : list of float
            estimated dummy input
        dummy_u_2s : list of float
            estimated dummy input
        raw_1s : list of float
            estimated constraint variable
        raw_2s : list of float
            estimated constraint variable
        N : int
            predict time step
        dt : float
            sampling time of system
        """
        F = []

        for i in range(N):
            F.append(self.R1 * u_1s[i] + lam_1s[i] + 2 * raw_1s[i] * (u_1s[i] + u_2s[i]))
            F.append(self.R2 * u_2s[i] + lam_2s[i] + 2 * raw_1s[i] * (u_1s[i] + u_2s[i]))
            F.append(self.R3 * u_3s[i] + lam_3s[i] + 2 * raw_2s[i] * u_3s[i])
            F.append(-0.01 + 2. * raw_1s[i] * dummy_u_1s[i])
            F.append(-0.01 + 2. * raw_2s[i] * dummy_u_2s[i])
            F.append((u_1s[i] + u_2s[i])**2 + dummy_u_1s[i]**2 - 1.**2)
            F.append(u_3s[i]**2 + dummy_u_2s[i]**2 - 1.5**2)
        
        return np.array(F)


