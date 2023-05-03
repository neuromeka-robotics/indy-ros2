'''
Created on 2019. 8. 19.

@author: YJHeo
@description: JSON program string maker
'''


import json

class WaypointParam:
    def __init__(self):
        pass


class MoveParam:
    def __init__(self):
        self.interpolator = 1
        self.ref_frame = dict(type=1, tref=[0, 0, 0, 0, 0, 0])
        self.tcp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.boundary = dict(velLevel=5, accLevel=5)
        self.stop_blend = True
        self.blend_option = dict(processLoop=False, constant=False)
        self.blend_raidus = 25.0
        self.offset = dict(type=0, pos=[0, 0, 0])
        self.abs_or_rel = 0
        self.tBase = 0


TYPE_STOP         = 1
TYPE_VAR_ASSIGN   = 3
TYPE_SMARTDO      = 4
TYPE_SMARTAO      = 5
TYPE_ENDTOOL_DO   = 6

TYPE_WAIT         = 22
TYPE_WAIT_FOR     = 23
TYPE_WAIT_FOR_DI  = 28
TYPE_TOOL_COMMAND = 40

TYPE_MOVE_HOME    = 100
TYPE_MOVE_ZERO    = 101
TYPE_JOINT_MOVE   = 102
TYPE_TASK_MOVE    = 103

TYPE_JOINT_SHAKE_MOVE = 106
TYPE_TASK_SHAKE_MOVE  = 106

TYPE_INDYCARE_COUNT      = 300
TYPE_INDYCARE_MONITORING = 301

POLICY_KEEP_PAUSE = 0
POLICY_RESUME_AFTER = 1
POLICY_STOP_PROGRAM = 2
POLICY_NO_COLLISION_DETECTION = 3


class JsonProgramComponent:
    def __init__(self, policy=0, resume_time=2,
                 var_name=None, var_value=None, var_type=None,
                 indycare_on=False, indycare_ip=None, caredata_name=None, caredata_type=None, caredata_target=None):
        # Init
        self.wp_id = 0
        self.program_id = 3
        self.cmd_id = 0

        # temp
        self.previous_method = '__init__'
        self.j_move = {}
        self.j_program = {}

        # Params
        self.mv_param = MoveParam()

        # Default blocks: configuration, variables, IndyCARE
        self.policy = policy
        self.time = resume_time
        self.collisionPolicy = dict(time=self.time, policy=self.policy)

        if indycare_on is False:
            self.indyCareInfo = {}
        elif isinstance(caredata_type, list):
            self.indycareData = []
            for i in range(len(caredata_name)):
                self.indycareData.append(dict(name=caredata_name[i], type=caredata_type[i], target=caredata_target[i]))
            # type : 0 (none), 1 (count), 2 (monitoring)
            # target (type=1): count (default=1)
            # target (type=2): 1 (AI00), 2 (AI01), 100 (CORE temperature)
            self.indyCareInfo = dict(useIndyCare=indycare_on, ipAddr="0.0.0.0", dataConfig=self.indycareData)
        else:
            self.indycareData = [dict(name=caredata_name, type=caredata_type, target=caredata_target)]
            self.indyCareInfo = dict(useIndyCare=indycare_on, ipAddr="0.0.0.0", dataConfig=self.indycareData)

        self.palletInfo = []
        self.toolInfo = []

        if var_name is None:
            self.varList = []
        elif isinstance(var_type, list):
            self.varList = []
            for i in range(len(var_name)):
                self.varList.append(dict(name=var_name[i], value=var_value[i], type=var_type[i]))
        else:
            self.varList = [(dict(name=var_name, value=var_value, type=var_type))]

        self.config = dict(type=999,
                           enable=True,
                           palletInfo=self.palletInfo,
                           toolInfo=self.toolInfo,
                           collisionPolicy=self.collisionPolicy,
                           indyCareInfo=self.indyCareInfo,
                           pId=0,
                           id=1
                           )

        self.var = dict(type=2,
                        varList=self.varList,
                        enable=True,
                        pId=0,
                        id=2
                        )

        # JSON program string
        self.wp_list = []
        self.program = [self.config, self.var]
        self.json_program = dict(program=self.program, moveList=[], wpList=[])

    def set_joint_dof(self, dof):
        self.joint_dof = 6
        pass
    # Set global variables
    def set_interpolator(self, interpolator):
        # 0: Time-based interpolator
        # 1: Trapezoidal interpolator
        self.mv_param.interpolator = interpolator

    def set_ref_frame(self, _type, t_ref=None, points=None):
        # 0: Base
        # 1: Direct
        # 2: Planar
        # 3: Circular
        if _type == 0:
            self.mv_param.ref_frame = dict(type=0)
        elif _type == 1:
            self.mv_param.ref_frame = dict(type=1, tref=t_ref)
        elif _type == 2:
            self.mv_param.ref_frame = dict(type=2, points=points)
        elif _type == 3:
            self.mv_param.ref_frame = dict(type=3, points=points)
        else:
            print('set_ref_frame: invalid type.')
            print('0: Base, 1: Direct, 2: Planar, 3: Circular')

    def set_tcp_frame(self, tcp):
        self.mv_param.tcp = tcp

    def set_velocity(self, vel):
        self.mv_param.boundary = dict(velLevel=vel, accLevel=vel)

    def set_joint_blend(self, rad):
        self.mv_param.stop_blend = False
        self.mv_param.blend_option = dict(processLoop=False, constant=False)
        self.mv_param.blend_raidus = rad

    def set_task_blend(self, rad):
        self.mv_param.stop_blend = False
        self.mv_param.blend_option = dict(processLoop=False, constant=False)
        self.mv_param.blend_raidus = rad

    def set_task_offset(self, _type, _pos):
        self.mv_param.offset = dict(type=_type, pos=_pos)

    def set_task_as_base(self):
        self.mv_param.tBase = 0

    def set_task_as_tcp(self):
        self.mv_param.tBase = 1

    def set_move_as_abs(self):
        self.mv_param.abs_or_rel = 0

    def set_joint_move_as_rel(self):
        self.mv_param.abs_or_rel = 1

    def set_task_move_as_rel(self):
        self.mv_param.abs_or_rel = 2

    # Make program dictionary
    def append_program(self):
        pass

    def append_wp_list(self):
        pass

    def append_move_list(self):
        pass

    # Add motion primitives
    def add_move_zero(self):
        _program = dict(type=TYPE_MOVE_ZERO,
                        enable=True,
                        pId=0,
                        id=self.program_id)

        self.program_id += 1
        self.json_program['program'].append(_program)
        self.previous_method = 'add_move_zero'

    def add_move_home(self):
        _program = dict(type=TYPE_MOVE_HOME,
                        enable=True,
                        pId=0,
                        id=self.program_id)

        self.program_id += 1
        self.json_program['program'].append(_program)
        self.previous_method = 'add_move_home'

    def add_stop(self):
        _program = dict(type=TYPE_STOP,
                        enable=True,
                        pId=0,
                        id=self.program_id)

        self.program_id += 1
        self.json_program['program'].append(_program)
        self.previous_method = 'add_stop'

    def add_var_assign(self, name, val, type):
        # 0: String
        # 1: Integer
        # 2: Floating Point
        # 3: Boolean
        # 5: Direct Variable
        _program = dict(type=TYPE_VAR_ASSIGN,
                        enable=True,
                        pId=0,
                        varList=[dict(name=name, value=val, type=type)],
                        id=self.program_id)

        self.program_id += 1
        self.json_program['program'].append(_program)
        self.previous_method = 'add_var_assign'

    def add_wait(self, time):
        _program = dict(type=TYPE_WAIT,
                        enable=True,
                        pId=0,
                        time=time,
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_wait'

    def add_wait_for(self, time, left_type, left_value, right_type, right_value, op):
        # Variable (type = 10)
        # Constant (type = 0,1,2,3 same as 'var_assign' / type = 4: DI, value = 1 or 0)
        # SmartDI (type = 11, value = 0~31 SmartDI idx)
        # SmartAI (type = 12, value = 0~1 SmartAI idx
        # op (0: ==, 1: !=, 2: >, 3: >=, 4: <, 5: <=)
        _program = dict(type=TYPE_WAIT_FOR,
                        enable=True,
                        pId=0,
                        time=time,
                        cond=dict(left=dict(type=left_type, value=left_value),
                                  right=dict(type=right_type, value=right_value),
                                  op=op),
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_wait_for'

    def add_wait_for_di(self, time, idx, val):
        _program = dict(type=TYPE_WAIT_FOR_DI,
                        enable=True,
                        pId=0,
                        time=time,
                        diList=[dict(idx=idx, value=val)],
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_wait_for_di'

    def add_digital_out(self, idx, val):
        _program = dict(type=TYPE_SMARTDO,
                        enable=True,
                        pId=0,
                        doList=[dict(idx=idx, value=val)],
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_digital_out'

    def add_analog_out(self, idx, val):
        _program = dict(type=TYPE_SMARTAO,
                        enable=True,
                        pId=0,
                        aoList=[dict(idx=idx, value=val)],
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_analog_out'

    def add_tool_command(self, tool_id, command):
        _program = dict(type=TYPE_TOOL_COMMAND,
                        enable=True,
                        pId=0,
                        toolCmd=dict(toolId=tool_id, cmdId=command),
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_tool_command'

    def add_endtool_do(self, type, value):
        _program = dict(type=TYPE_ENDTOOL_DO,
                        enable=True,
                        pId=0,
                        endtoolDoList=[dict(value=value, type=type)],
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_endtool_do'

    def add_joint_move(self, joint_pos):

        if (self.previous_method == 'add_joint_move_to' and self.mv_param.abs_or_rel == 0) \
                or (self.previous_method == 'add_joint_move_by' and self.mv_param.abs_or_rel == 1):
            # Append move list
            _new_wp_move = dict(t=2, id=self.wp_id)
            self.json_program['moveList'][-1]['wpList'].append(_new_wp_move)

        else:
            # Append move list
            _j_move = dict(type=TYPE_JOINT_MOVE,
                           name='jmove-%02d' % self.program_id,
                           intpl=self.mv_param.interpolator,
                           tcp=self.mv_param.tcp,
                           refFrame=self.mv_param.ref_frame,
                           boundary=self.mv_param.boundary,
                           blendOpt=self.mv_param.blend_option,
                           wpList=[dict(t=2, id=self.wp_id)])
            self.json_program['moveList'].append(_j_move)

            # Append program list
            _j_program = dict(pId=0,
                              enable=True,
                              type=TYPE_JOINT_MOVE,
                              name='jmove-%02d' % self.program_id,
                              id=self.program_id)

            self.program_id += 1
            self.json_program['program'].append(_j_program)

        # Append waypoint list
        _new_wp_wp = dict(id=self.wp_id,
                          name='jmove-%02d-%02d' % (self.program_id - 1, self.wp_id),
                          type=self.mv_param.abs_or_rel,
                          tBase=self.mv_param.tBase,
                          stopBlend=self.mv_param.stop_blend,
                          blendRadius=self.mv_param.blend_raidus,
                          q=joint_pos,
                          p=[0, 0, 0, 0, 0, 0])
        self.wp_id += 1
        self.json_program['wpList'].append(_new_wp_wp)
        if self.mv_param.abs_or_rel == 0:
            self.previous_method = 'add_joint_move_to'
        elif self.mv_param.abs_or_rel == 1:
            self.previous_method = 'add_joint_move_by'

    def add_task_move(self, task_pos):
        # Get move params
        # mv_param = MoveParam()
        if (self.previous_method == 'add_task_move_to' and self.mv_param.abs_or_rel == 0) \
                or (self.previous_method == 'add_task_move_by' and self.mv_param.abs_or_rel == 2):
            # Append move list
            _new_wp_move = dict(t=2, id=self.wp_id)
            self.json_program['moveList'][-1]['wpList'].append(_new_wp_move)
        else:
            # Append move list
            _t_move = dict(type=TYPE_TASK_MOVE,
                           name='tmove-%02d' % self.program_id,
                           intpl=self.mv_param.interpolator,
                           tcp=self.mv_param.tcp,
                           refFrame=self.mv_param.ref_frame,
                           boundary=self.mv_param.boundary,
                           blendOpt=self.mv_param.blend_option,
                           wpList=[dict(t=2, id=self.wp_id)],
                           offset=self.mv_param.offset)
            self.json_program['moveList'].append(_t_move)

            # Append program list
            _j_program = dict(pId=0,
                              enable=True,
                              type=TYPE_TASK_MOVE,
                              name='tmove-%02d' % self.program_id,
                              id=self.program_id)

            self.program_id += 1
            self.json_program['program'].append(_j_program)

        # Append waypoint list
        _new_wp_wp = dict(id=self.wp_id,
                          name='tmove-%02d-%02d' % (self.program_id - 1, self.wp_id),
                          type=self.mv_param.abs_or_rel,
                          tBase=self.mv_param.tBase,
                          stopBlend=self.mv_param.stop_blend,
                          blendRadius=self.mv_param.blend_raidus,
                          q=[0, 0, 0, 0, 0, 0],
                          p=task_pos)

        self.wp_id += 1
        self.json_program['wpList'].append(_new_wp_wp)
        if self.mv_param.abs_or_rel == 0:
            self.previous_method = 'add_task_move_to'
        elif self.mv_param.abs_or_rel == 2:
            self.previous_method = 'add_task_move_by'

    def add_joint_shake_move(self, cycle_num, shake_strength, shake_time, shake_ratio, side):
        pass

    def add_task_shake_move(self, task_pos, vel, blend, cycle_num, shake_strength, shake_time, shake_ratio, side):
        # Get move params
        self.set_velocity(vel)
        self.set_task_blend(blend)
        if self.previous_method == 'add_task_shake_move':
            # Append move list
            _new_wp_move = dict(t=2, id=self.wp_id)
            self.json_program['moveList'][-1]['wpList'].append(_new_wp_move)
        else:
            # Append move list
            _t_move = dict(type=TYPE_TASK_SHAKE_MOVE,
                           name='tsmove-%02d' % self.program_id,
                           intpl=self.mv_param.interpolator,
                           tcp=self.mv_param.tcp,
                           refFrame=self.mv_param.ref_frame,
                           boundary=self.mv_param.boundary,
                           blendOpt=self.mv_param.blend_option,
                           wpList=[dict(t=2, id=self.wp_id)],
                           offset=self.mv_param.offset,
                           shakingVar=dict(cyclicRate=cycle_num,
                                            shakeStrength=shake_strength,
                                            shakeTime=shake_time,
                                            shakeStartRatio=shake_ratio,
                                            side=side))
            self.json_program['moveList'].append(_t_move)

            # Append program list
            _j_program = dict(pId=0,
                              enable=True,
                              type=TYPE_TASK_MOVE,
                              name='tsmove-%02d' % self.program_id,
                              id=self.program_id)

            self.program_id += 1
            self.json_program['program'].append(_j_program)

        # Append waypoint list
        _new_wp_wp = dict(id=self.wp_id,
                          name='tsmove-%02d-%02d' % (self.program_id - 1, self.wp_id),
                          type=self.mv_param.abs_or_rel,
                          tBase=self.mv_param.tBase,
                          stopBlend=self.mv_param.stop_blend,
                          blendRadius=self.mv_param.blend_raidus,
                          q=[0, 0, 0, 0, 0, 0],
                          p=task_pos)

        self.wp_id += 1
        self.json_program['wpList'].append(_new_wp_wp)
        self.previous_method = 'add_task_shake_move'


    # Warpping functions (only velocity and blending can be parameterized)
    def add_joint_move_to(self, q, vel=3, blend=5):
        self.set_velocity(vel)
        self.set_joint_blend(blend)
        self.set_move_as_abs()
        self.add_joint_move(q)

    def add_joint_move_by(self, q, vel=3, blend=5):
        # self.set_velocity(vel)
        # self.set_joint_blend(blend)
        # self.set_joint_move_as_rel()
        # self.add_joint_move(q)
        pass

    def add_task_move_to(self, p, vel=3, blend=0.05):
        self.set_velocity(vel)
        self.set_task_blend(blend)
        self.set_move_as_abs()
        self.add_task_move(p)

    def add_task_move_by(self, p, vel=3, blend=0.05):
        self.set_velocity(vel)
        self.set_task_blend(blend)
        self.set_task_move_as_rel()
        self.add_task_move(p)

    # IndyCARE Reporting
    def add_indycare_count(self, idx):
        _program = dict(type=TYPE_INDYCARE_COUNT,
                        enable=True,
                        pId=0,
                        careIdx=idx,
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_indycare_count'

    def add_indycare_monitoring(self, idx):
        _program = dict(type=TYPE_INDYCARE_MONITORING,
                        enable=True,
                        pId=0,
                        careIdx=idx,
                        id=self.program_id)

        self.json_program['program'].append(_program)
        self.program_id += 1
        self.previous_method = 'add_indycare_monitoring'

    def program_done(self):
        json_prog = json.dumps(self.json_program)
        return json_prog

    def get_program_json(self):
        return self.json_program

class PickNPlace:
    def __init__(self, take_pos):
        # High-level program
        self.indy_program = JsonProgramComponent()
        self.motion_primitive = []
        self.program_sequence = []

        # Variables
        self.take_pos = take_pos

    def pick_motion(self, obj_pos, pick_joint_pos):
        obj_pos_pre = obj_pos
        obj_pos_pre[2] += 0.2

        self.indy_program.add_joint_move_to(self.take_pos)
        self.indy_program.add_joint_move_to(pick_joint_pos)

        self.indy_program.add_task_move_to(obj_pos_pre)
        self.indy_program.add_task_move_to(obj_pos)
        self.indy_program.add_digital_out(8, 1)
        self.indy_program.add_task_move_to(obj_pos_pre)

    def place_motion(self, pre_joint_pos, place_pos):
        place_pos_pre = place_pos
        place_pos_pre[2] += 0.2

        self.indy_program.add_joint_move_to(pre_joint_pos)

        self.indy_program.add_task_move_to(place_pos_pre)
        self.indy_program.add_task_move_to(place_pos)
        self.indy_program.add_digital_out(8, 0)
        self.indy_program.add_task_move_to(place_pos_pre)

        self.indy_program.add_move_home()

    def pick_and_place(self, obj_pos, pick_joint_pos, pre_joint_pos, place_pos):
        self.pick_motion(obj_pos, pick_joint_pos)
        self.place_motion(pre_joint_pos, place_pos)

