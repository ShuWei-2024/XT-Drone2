#!/usr/bin/env python3
"""Companion controls for RViz; publishes UI requests, never flight setpoints."""
import json
import sys
import time
import uuid
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from PyQt5 import QtCore, QtWidgets


class Controls(QtWidgets.QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.client = uuid.uuid4().hex
        self.request = 0
        self.state = None
        self.receipt = 0.
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/astra/ui/status', self.update_status, qos)
        self.command_pub = node.create_publisher(String, '/astra/ui/command', 10)
        self.goal_pub = node.create_publisher(PoseStamped, '/astra/ui/goal', 10)
        self.setWindowTitle('Astra · 三维航点飞行控制')
        self.setGeometry(1380, 60, 390, 670)
        layout = QtWidgets.QVBoxLayout(self)
        self.status_label = QtWidgets.QLabel('等待仿真和定位就绪…')
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)
        help_label = QtWidgets.QLabel('RViz：鼠标旋转三维地图；G 添加 XY 航点。\n也可在下方输入坐标。航点编号表示执行顺序。')
        help_label.setWordWrap(True); layout.addWidget(help_label)
        self.height = QtWidgets.QLabel('巡航高度：等待场景配置')
        layout.addWidget(self.height)
        self.list = QtWidgets.QListWidget(); layout.addWidget(self.list)
        row = QtWidgets.QHBoxLayout()
        self.x, self.y = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
        for box, name in ((self.x, 'X: '), (self.y, 'Y: ')):
            box.setRange(-10, 10); box.setDecimals(2); box.setSingleStep(.1); box.setPrefix(name)
            row.addWidget(box)
        layout.addLayout(row)
        self.add = QtWidgets.QPushButton('添加坐标航点')
        self.add.clicked.connect(self.add_goal); layout.addWidget(self.add)
        row = QtWidgets.QHBoxLayout()
        self.remove = QtWidgets.QPushButton('删除末点'); self.clear = QtWidgets.QPushButton('清空航点')
        self.remove.clicked.connect(lambda: self.send('remove_last'))
        self.clear.clicked.connect(lambda: self.send('clear'))
        row.addWidget(self.remove); row.addWidget(self.clear); layout.addLayout(row)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItem('最后一个航点：避障返航，然后降落', 'return_home')
        self.mode.addItem('最后一个航点：降落，不返回', 'land')
        self.mode.addItem('最后一个航点：悬停，等待追加目标', 'hold')
        layout.addWidget(self.mode)
        self.wait = QtWidgets.QSpinBox(); self.wait.setRange(30, 120); self.wait.setValue(60)
        self.wait.setPrefix('等待／暂停超时：'); self.wait.setSuffix(' 秒，之后结束或返航')
        layout.addWidget(self.wait)
        self.start = QtWidgets.QPushButton('开始任务（起飞）')
        self.start.clicked.connect(lambda: self.send('start', end_action=self.mode.currentData(), wait_seconds=self.wait.value()))
        layout.addWidget(self.start)
        self.pause = QtWidgets.QPushButton('暂停／继续')
        self.pause.clicked.connect(lambda: self.send('resume' if self.state and self.state['paused'] else 'pause'))
        layout.addWidget(self.pause)
        self.land = QtWidgets.QPushButton('结束飞行并受控降落')
        self.land.setStyleSheet('background:#a43b35;color:white;padding:8px;')
        self.land.clicked.connect(lambda: self.send('land')); layout.addWidget(self.land)
        self.return_home = QtWidgets.QPushButton('结束任务：避障返航并降落')
        self.return_home.clicked.connect(lambda: self.send('return_home')); layout.addWidget(self.return_home)
        note = QtWidgets.QLabel('地面选点后点开始任务才起飞。空中可追加航点。\n青色边线／填色：静态可选航点，点击仍检查实时障碍。\n绿色：已检查降落区；蓝色：巡航通行区（若场景支持）。\n通行区不能直接降落，需返航；背景模型仅供观察。\n失联时按场景保护策略结束；官方 RTL 不自动避障。')
        note.setWordWrap(True); layout.addWidget(note)
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.spin); self.timer.start(20)
        self.heartbeat = QtCore.QTimer(self); self.heartbeat.timeout.connect(lambda: self.send('heartbeat')); self.heartbeat.start(500)
        self.refresh()

    def update_status(self, msg):
        try:
            self.state = json.loads(msg.data)
            self.receipt = time.monotonic()
        except (ValueError, TypeError):
            return
        self.refresh()

    def refresh(self):
        s = self.state
        fresh = s is not None and time.monotonic()-self.receipt < 2.
        editing = fresh and s['editing'] and not s['closed']
        started = fresh and s['started']
        self.add.setEnabled(editing)
        self.start.setEnabled(editing and not started and bool(s['goals']))
        self.remove.setEnabled(editing and not started)
        self.clear.setEnabled(editing and not started)
        self.mode.setEnabled(not started); self.wait.setEnabled(not started)
        self.pause.setEnabled(editing and started)
        self.land.setEnabled(fresh and not s['closed'])
        self.return_home.setEnabled(editing and started)
        if not fresh:
            self.status_label.setText('等待控制状态／状态已过期，禁止新操作')
            return
        phases = {'rviz_goal_selection':'编辑航点', 'ego_execution':'飞向航点',
                  'waiting_waypoint':'到点悬停，等待追加航点', 'mission_paused':'暂停保持',
                  'land':'降落', 'climb':'起飞', 'alignment_capture':'坐标对齐',
                  'preflight':'起飞前检查', 'prestream':'准备起飞', 'settle':'等待稳定',
                  'heading_alignment':'航向校准', 'cruise_descent':'下降至巡航高度',
                  'cruise_settle':'巡航准备', 'initial_yaw_agreement':'检查航向对齐',
                  'endpoint_hold':'到点保持', 'fault_hold':'中止并保持', 'native_rtl':'PX4 官方返航降落'}
        messages={'READY: add waypoints, then press Start':'已就绪：添加航点后点击开始任务',
                  'REJECTED: Goal lacks planner detour reserve; select farther inside the safe region':
                  '航点太靠近安全区域边缘，规划器无法接收；请向区域内部重新选点',
                  'REJECTED: Last waypoint is transit only; choose return home or hold':
                  '末航点只能通行，不能原地下降；请选择返航或悬停',
                  'Starting mission':'任务已开始，准备起飞',
                  'Flying: additional waypoints join the queue':'正在飞行，可追加后续航点',
                  'Waiting for another waypoint':'已到点：追加航点后继续飞行',
                  'Returning home through planner':'正在由 EGO 规划返航',
                  'Returned home: landing':'已返航：在起飞区降落',
                  'Mission completed: landing here':'任务完成：在最后航点降落', 'PAUSED':'保持当前位置'}
        self.status_label.setText(phases.get(s['phase'],s['phase'])+'\n'+messages.get(s['message'],s['message']))
        self.height.setText('%s\n巡航：%.2f m · 校准爬升：%.2f m\n航点范围：起飞点周围 %.1f m'%(
            s.get('scene_name','室内场景'),s['height_m'],s.get('alignment_height_m',1.60),s.get('goal_radius_m',3.)))
        self.pause.setText('继续并重新规划' if s['paused'] else '暂停并保持当前位置')
        self.list.clear()
        for i, (x,y) in enumerate(s['goals']):
            state = '已到达' if i<s['completed'] else '待飞'
            self.list.addItem('%d. X=%+.2f Y=%+.2f · %s'%(i+1,x+s['origin'][0],y+s['origin'][1],state))

    def send(self, command, **fields):
        if self.state is None or time.monotonic()-self.receipt>2.:
            return
        self.request += 1
        fields.update(command=command, session=self.state['session'], client=self.client,
                      request_id=self.request, stamp_ns=self.node.get_clock().now().nanoseconds)
        self.command_pub.publish(String(data=json.dumps(fields)))

    def add_goal(self):
        msg = PoseStamped(); msg.header.frame_id='camera_init'
        msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.pose.position.x=self.x.value(); msg.pose.position.y=self.y.value()
        msg.pose.orientation.w=1.
        self.goal_pub.publish(msg)

    def spin(self):
        if not rclpy.ok():
            QtWidgets.QApplication.quit()
            return
        for _ in range(4):
            rclpy.spin_once(self.node, timeout_sec=0.)
        if time.monotonic()-self.receipt>2.:
            self.refresh()

    def closeEvent(self, event):
        self.send('land')
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=.01)
        event.accept()


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('astra_rviz_mission_panel', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    app=QtWidgets.QApplication(sys.argv[:1])
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    app.setStyleSheet('QWidget {font-size:14px;} QPushButton {padding:5px;}')
    window=Controls(node);window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
