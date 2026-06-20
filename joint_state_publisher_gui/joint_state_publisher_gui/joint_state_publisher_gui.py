# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import os
import random
import signal
import sys
import threading

import rclpy

import yaml

from python_qt_binding.QtCore import pyqtSlot
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import QApplication
from python_qt_binding.QtWidgets import QFormLayout
from python_qt_binding.QtWidgets import QGridLayout
from python_qt_binding.QtWidgets import QGroupBox
from python_qt_binding.QtWidgets import QHBoxLayout
from python_qt_binding.QtWidgets import QInputDialog
from python_qt_binding.QtWidgets import QLabel
from python_qt_binding.QtWidgets import QLineEdit
from python_qt_binding.QtWidgets import QMainWindow
from python_qt_binding.QtWidgets import QPushButton
from python_qt_binding.QtWidgets import QSlider
from python_qt_binding.QtWidgets import QScrollArea
from python_qt_binding.QtWidgets import QVBoxLayout
from python_qt_binding.QtWidgets import QWidget

from joint_state_publisher.joint_state_publisher import JointStatePublisher

from joint_state_publisher_gui.flow_layout import FlowLayout

RANGE = 10000
LINE_EDIT_WIDTH = 60
SLIDER_WIDTH = 200
INIT_NUM_SLIDERS = 7  # Initial number of sliders to show in window

# Defined by style - currently using the default style
DEFAULT_WINDOW_MARGIN = 11
DEFAULT_CHILD_MARGIN = 9
DEFAULT_BTN_HEIGHT = 25
DEFAULT_SLIDER_HEIGHT = 64  # Is the combination of default heights in Slider

# Calculate default minimums for window sizing
MIN_WIDTH = SLIDER_WIDTH + DEFAULT_CHILD_MARGIN * 4 + DEFAULT_WINDOW_MARGIN * 2
MIN_HEIGHT = DEFAULT_BTN_HEIGHT * 2 + DEFAULT_WINDOW_MARGIN * 2 + DEFAULT_CHILD_MARGIN * 2

# Clean flat restyle for the panel — purely cosmetic, no behavior change.
GUI_STYLESHEET = """
QMainWindow, QWidget { background-color: #f4f5f7; color: #2b2f36; }
QWidget { font-family: "Helvetica", "Segoe UI", sans-serif; font-size: 12px; }

QGroupBox {
    border: 1px solid #dcdfe4;
    border-radius: 8px;
    margin-top: 14px;
    padding: 10px;
    background-color: #fbfcfd;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 6px;
    color: #6b7480;
    font-weight: 600;
}

QPushButton {
    background-color: #ffffff;
    border: 1px solid #cfd4db;
    border-radius: 6px;
    padding: 6px 12px;
    min-height: 16px;
}
QPushButton:hover   { background-color: #eef4fd; border-color: #4a90d9; }
QPushButton:pressed { background-color: #d8e6f8; border-color: #357abd; }

QPushButton#primaryBtn {
    background-color: #2f80ed; border: 1px solid #2f80ed; color: #ffffff; font-weight: 600;
}
QPushButton#primaryBtn:hover   { background-color: #2d76d6; }
QPushButton#primaryBtn:pressed { background-color: #2769bf; }

QPushButton#poseBtn {
    background-color: #eef2f7; border: 1px solid #cdd5df; color: #34404d; font-weight: 600;
}
QPushButton#poseBtn:hover   { background-color: #e3ebf5; border-color: #4a90d9; }
QPushButton#poseBtn:pressed { background-color: #d3e0f1; }

QLineEdit {
    background-color: #ffffff; border: 1px solid #d3d8df;
    border-radius: 4px; padding: 2px 5px; color: #1f2933;
}

QSlider::groove:horizontal { height: 4px; background: #d6dbe1; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #2f80ed; border-radius: 2px; }
QSlider::add-page:horizontal { background: #d6dbe1; border-radius: 2px; }
QSlider::handle:horizontal {
    background: #ffffff; border: 1px solid #9aa4b0;
    width: 14px; margin: -6px 0; border-radius: 7px;
}
QSlider::handle:horizontal:hover { border-color: #2f80ed; }

QScrollArea { border: none; background: transparent; }
"""


class Slider(QWidget):
    def __init__(self, name):
        super().__init__()

        self.joint_layout = QVBoxLayout()
        self.row_layout = QHBoxLayout()

        font = QFont("Helvetica", 9, QFont.Bold)
        self.label = QLabel(name)
        self.label.setFont(font)
        self.row_layout.addWidget(self.label)

        self.display = QLineEdit("0.00")
        self.display.setAlignment(Qt.AlignRight)
        self.display.setFont(font)
        self.display.setReadOnly(True)
        self.display.setFixedWidth(LINE_EDIT_WIDTH)
        self.row_layout.addWidget(self.display)

        self.joint_layout.addLayout(self.row_layout)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setFont(font)
        self.slider.setRange(0, RANGE)
        self.slider.setValue(int(RANGE / 2))
        self.slider.setFixedWidth(SLIDER_WIDTH)

        self.joint_layout.addWidget(self.slider)

        self.setLayout(self.joint_layout)

    def remove(self):
        self.joint_layout.removeWidget(self.slider)
        self.slider.setParent(None)

        self.row_layout.removeWidget(self.display)
        self.display.setParent(None)

        self.row_layout.removeWidget(self.label)
        self.label.setParent(None)

        self.row_layout.setParent(None)


class JointStatePublisherGui(QMainWindow):
    sliderUpdateTrigger = Signal()
    initialize = Signal()

    def __init__(self, title, jsp):
        super(JointStatePublisherGui, self).__init__()

        self.joint_map = {}

        self.setWindowTitle(title)
        self.setStyleSheet(GUI_STYLESHEET)

        # Button for randomizing the sliders
        self.rand_button = QPushButton('Randomize', self)
        self.rand_button.clicked.connect(self.randomizeEvent)

        # Button for centering the sliders
        self.ctr_button = QPushButton('Center', self)
        self.ctr_button.clicked.connect(self.centerEvent)

        # Named-pose buttons (opt-in): if JSP_NAMED_POSES points at a YAML file of
        # {pose_name: {joint: value}}, add a "Save Pose..." button plus one button
        # per saved pose that snaps the sliders to it. No file -> no extra buttons.
        self.named_poses = self._load_named_poses()
        self.pose_buttons = []
        self.save_button = None
        if os.environ.get('JSP_NAMED_POSES'):
            self.save_button = QPushButton('Save Pose...', self)
            self.save_button.clicked.connect(self.savePoseEvent)
        for pose_name in self.named_poses:
            self.pose_buttons.append(self._make_pose_button(pose_name))

        # Pose-editing buttons (opt-in, same gate as named poses). Each edit
        # snapshots the sliders to an undo stack first, then mutates. The
        # robot-specific 'Plant Wheels' action is loaded LAZILY from the optional
        # companion module named by JSP_POSE_ACTION_MODULE (absent -> no button).
        self.undo_stack = []
        self.edit_buttons = []
        if os.environ.get('JSP_NAMED_POSES'):
            self._add_edit_button('Symmetrize L->R', self.symmetrizeEvent)
            self._add_edit_button('Mirror L<->R', self.mirrorEvent)
            self._add_edit_button('Flatten Twist', self.flattenTwistEvent)
            self._add_edit_button('Zero Legs', self.zeroLegsEvent)
            self.plant_fn = self._load_plant_action()
            if self.plant_fn is not None:
                self._add_edit_button('Plant Wheels', self.plantWheelsEvent)
            # Undo is NOT wrapped (it IS the restore); placed last.
            self.undo_button = QPushButton('Undo', self)
            self.undo_button.clicked.connect(self.undoEvent)
            self.edit_buttons.append(self.undo_button)

        # Scroll area widget contents - layout
        self.scroll_layout = FlowLayout()

        # Scroll area widget contents
        self.scroll_widget = QWidget()
        self.scroll_widget.setLayout(self.scroll_layout)

        # Scroll area for sliders
        self.scroll_area = QScrollArea()
        self.scroll_area.setWidgetResizable(True)
        self.scroll_area.setWidget(self.scroll_widget)

        # Top control band: compact labeled groups side by side (Actions / Edit
        # pose / Named poses) instead of one tall column of full-width buttons.
        controls_row = QHBoxLayout()
        controls_row.setSpacing(8)

        actions_group = QGroupBox("Actions")
        actions_col = QVBoxLayout()
        actions_col.setSpacing(6)
        actions_col.addWidget(self.rand_button)
        actions_col.addWidget(self.ctr_button)
        if self.save_button is not None:
            self.save_button.setObjectName("primaryBtn")
            actions_col.addWidget(self.save_button)
        actions_col.addStretch(1)
        actions_group.setLayout(actions_col)
        controls_row.addWidget(actions_group)

        if self.edit_buttons:
            controls_row.addWidget(
                self._button_grid("Edit pose", self.edit_buttons, columns=2))

        if self.pose_buttons:
            for btn in self.pose_buttons:
                btn.setObjectName("poseBtn")
            controls_row.addWidget(
                self._button_grid("Named poses", self.pose_buttons, columns=2))

        controls_row.addStretch(1)

        # Main layout: control band on top, sliders (scroll area) filling below.
        self.main_layout = QVBoxLayout()
        self.main_layout.setSpacing(8)
        self.main_layout.addLayout(controls_row)
        self.main_layout.addWidget(self.scroll_area, 1)

        # central widget
        self.central_widget = QWidget()
        self.central_widget.setLayout(self.main_layout)
        self.setCentralWidget(self.central_widget)

        self.jsp = jsp
        self.jsp.set_source_update_cb(self.sliderUpdateCb)
        self.jsp.set_robot_description_update_cb(self.initializeCb)

        self.running = True
        self.sliders = {}

        # Setup signal for initializing the window
        self.initialize.connect(self.initializeSliders)
        # Set up a signal for updating the sliders based on external joint info
        self.sliderUpdateTrigger.connect(self.updateSliders)

        # Tell self to draw sliders in case the JointStatePublisher already has a robot_description
        self.initialize.emit()

    def initializeSliders(self):
        self.joint_map = {}

        for sl, _ in self.sliders.items():
            self.scroll_layout.removeWidget(sl)
            sl.remove()

        ### Generate sliders ###
        for name in self.jsp.joint_list:
            if name not in self.jsp.free_joints:
                continue
            joint = self.jsp.free_joints[name]

            if joint['min'] == joint['max']:
                continue

            slider = Slider(name)

            self.joint_map[name] = {'display': slider.display, 'slider': slider.slider, 'joint': joint}

            self.scroll_layout.addWidget(slider)
            # Connect to the signal provided by QSignal
            slider.slider.valueChanged.connect(lambda event,name=name: self.onSliderValueChangedOne(name))

            self.sliders[slider] = slider

        # Set zero positions read from parameters
        self.centerEvent(None)

        # Set size of min size of window based on number of sliders.
        if len(self.sliders) >= INIT_NUM_SLIDERS:  # Limits min size to show INIT_NUM_SLIDERS
            num_sliders = INIT_NUM_SLIDERS
        else:
            num_sliders = len(self.sliders)
        scroll_layout_height = num_sliders * DEFAULT_SLIDER_HEIGHT
        scroll_layout_height += (num_sliders + 1) * DEFAULT_CHILD_MARGIN
        self.setMinimumSize(MIN_WIDTH, scroll_layout_height + MIN_HEIGHT)

        self.sliderUpdateTrigger.emit()

    def sliderUpdateCb(self):
        self.sliderUpdateTrigger.emit()

    def initializeCb(self):
        self.initialize.emit()

    def onSliderValueChangedOne(self, name):
        # A slider value was changed, but we need to change the joint_info metadata.
        joint_info = self.joint_map[name]
        slidervalue = joint_info['slider'].value()
        joint = joint_info['joint']
        joint['position'] = self.sliderToValue(slidervalue, joint)
        joint_info['display'].setText("%.3f" % joint['position'])

    @pyqtSlot()
    def updateSliders(self):
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            slidervalue = self.valueToSlider(joint['position'], joint)
            joint_info['slider'].setValue(slidervalue)

    def centerEvent(self, event):
        self.jsp.get_logger().info("Centering")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(self.valueToSlider(joint['zero'], joint))

    def randomizeEvent(self, event):
        self.jsp.get_logger().info("Randomizing")
        for name, joint_info in self.joint_map.items():
            joint = joint_info['joint']
            joint_info['slider'].setValue(
                self.valueToSlider(random.uniform(joint['min'], joint['max']), joint))

    # ---- pose-editing helpers (shared by the edit buttons) ----

    def _add_edit_button(self, label, handler):
        btn = QPushButton(label, self)
        btn.clicked.connect(handler)
        self.edit_buttons.append(btn)

    def _button_grid(self, title, buttons, columns=2):
        """Lay buttons out in a wrapping grid inside a titled group box."""
        group = QGroupBox(title)
        grid = QGridLayout()
        grid.setSpacing(6)
        for i, btn in enumerate(buttons):
            grid.addWidget(btn, i // columns, i % columns)
        group.setLayout(grid)
        return group

    def _capture_pose(self):
        """Current slider values, keyed by joint NAME (survives a URDF reload)."""
        return {name: self.sliderToValue(ji['slider'].value(), ji['joint'])
                for name, ji in self.joint_map.items()}

    def _apply_pose(self, pose):
        """Write a {name: value} dict to the sliders, clamped to each joint's own
        limits. The RESTORE primitive — must NOT push to the undo stack."""
        for name, ji in self.joint_map.items():
            if name not in pose:
                continue
            joint = ji['joint']
            value = max(joint['min'], min(joint['max'], float(pose[name])))
            ji['slider'].setValue(self.valueToSlider(value, joint))

    def _apply_edit(self, fn):
        """Snapshot the sliders (for undo), then run the mutating edit `fn`."""
        snap = self._capture_pose()
        fn()
        if snap != self._capture_pose():            # skip no-op edits
            self.undo_stack.append(snap)
            del self.undo_stack[:-50]               # cap depth

    # L<->R joints that NEGATE under a true mirror (lateral/twist DOF); leg
    # joints share the same +Y axis & limits so they mirror by plain copy.
    _MIRROR_NEGATE = ('base_y', 'base_roll', 'base_yaw')

    def symmetrizeEvent(self, event=None):
        """Copy every L_* value onto its R_* twin (legs match the left side)."""
        def edit():
            pose = self._capture_pose()
            new = dict(pose)
            for name in pose:
                if name.startswith('L_') and ('R_' + name[2:]) in self.joint_map:
                    new['R_' + name[2:]] = pose[name]
            self._apply_pose(new)
        self.jsp.get_logger().info("Symmetrize L->R")
        self._apply_edit(edit)

    def mirrorEvent(self, event=None):
        """Swap L_* <-> R_* (atomic from the snapshot); negate lateral/twist DOF."""
        def edit():
            pose = self._capture_pose()
            new = dict(pose)
            for name in pose:
                if name.startswith('L_'):
                    r = 'R_' + name[2:]
                    if r in self.joint_map:
                        new[name], new[r] = pose[r], pose[name]
            for n in self._MIRROR_NEGATE:           # true mirror negates these
                if n in new:
                    new[n] = -new[n]
            self._apply_pose(new)
        self.jsp.get_logger().info("Mirror L<->R")
        self._apply_edit(edit)

    def flattenTwistEvent(self, event=None):
        """Zero base_roll + base_yaw (remove sideways twist)."""
        def edit():
            pose = self._capture_pose()
            for n in ('base_roll', 'base_yaw'):
                if n in pose:
                    pose[n] = 0.0
            self._apply_pose(pose)
        self.jsp.get_logger().info("Flatten twist")
        self._apply_edit(edit)

    def zeroLegsEvent(self, event=None):
        """Zero the leg joints (hip/knee/ankle, both sides)."""
        def edit():
            pose = self._capture_pose()
            for n in list(pose):
                if any(k in n for k in ('hip', 'knee', 'ankle')):
                    pose[n] = 0.0
            self._apply_pose(pose)
        self.jsp.get_logger().info("Zero legs")
        self._apply_edit(edit)

    def plantWheelsEvent(self, event=None):
        """Robot-specific: set base_z so the lowest wheel sits on the ground,
        via the lazily-loaded companion (pure: pose dict -> {base_z}). No I/O."""
        if not self.joint_map.get('base_z'):
            self.jsp.get_logger().warn("Plant Wheels: no 'base_z' slider")
            return

        def edit():
            pose = self._capture_pose()
            out = self.plant_fn(pose)               # {"base_z": value}
            if 'base_z' in out:
                pose['base_z'] = out['base_z']
                self._apply_pose(pose)
        self.jsp.get_logger().info("Plant wheels")
        self._apply_edit(edit)

    def undoEvent(self, event=None):
        """Restore the slider values from before the last edit."""
        if not self.undo_stack:
            self.jsp.get_logger().info("Nothing to undo")
            return
        self._apply_pose(self.undo_stack.pop())

    def _load_plant_action(self):
        """Lazy-import the optional robot-specific plant_wheels(pose)->{base_z}.
        Module path from JSP_POSE_ACTION_MODULE='pkg.module:function'. Absent or
        unimportable -> None (no Plant Wheels button), never an error."""
        spec = os.environ.get('JSP_POSE_ACTION_MODULE')
        if not spec or ':' not in spec:
            return None
        mod_name, fn_name = spec.split(':', 1)
        try:
            import importlib
            mod = importlib.import_module(mod_name)
            return getattr(mod, fn_name)
        except Exception as exc:  # noqa: BLE001 - optional; never break the GUI
            # self.jsp may not exist yet (this runs from __init__), so guard it —
            # an unimportable action must degrade to "no button", never a crash.
            msg = "plant action not loaded: %s" % exc
            jsp = getattr(self, "jsp", None)
            if jsp is not None:
                jsp.get_logger().warn(msg)
            else:
                print("[joint_state_publisher_gui] " + msg)
            return None

    def setPoseEvent(self, pose_name):
        """Snap the sliders to a named pose (clamped to each joint's limits)."""
        self.jsp.get_logger().info("Setting pose '%s'" % pose_name)
        pose = self.named_poses.get(pose_name, {})
        for name, joint_info in self.joint_map.items():
            if name not in pose:
                continue
            joint = joint_info['joint']
            value = max(joint['min'], min(joint['max'], float(pose[name])))
            joint_info['slider'].setValue(self.valueToSlider(value, joint))

    def savePoseEvent(self, event):
        """Prompt for a name, store the CURRENT sliders as a named pose, add a
        button for it, and write the YAML back to disk."""
        name, ok = QInputDialog.getText(self, 'Save Pose', 'Pose name:')
        name = name.strip()
        if not ok or not name:
            return
        pose = {}
        for joint_name, joint_info in self.joint_map.items():
            value = self.sliderToValue(joint_info['slider'].value(), joint_info['joint'])
            pose[joint_name] = round(float(value), 6)
        new = name not in self.named_poses
        self.named_poses[name] = pose
        self._save_named_poses()
        if new:                                  # add a recall button live
            btn = self._make_pose_button(name)
            self.pose_buttons.append(btn)
            # insert just after the Save button in the layout
            idx = self.main_layout.indexOf(self.save_button) + len(self.pose_buttons)
            self.main_layout.insertWidget(idx, btn)
        self.jsp.get_logger().info("Saved pose '%s'" % name)

    def _make_pose_button(self, pose_name):
        """A recall button that snaps the sliders to the named pose."""
        btn = QPushButton(pose_name, self)
        btn.clicked.connect(
            lambda _checked=False, n=pose_name: self.setPoseEvent(n))
        return btn

    def _save_named_poses(self):
        """Write self.named_poses back to the JSP_NAMED_POSES YAML file."""
        path = os.environ.get('JSP_NAMED_POSES')
        if not path:
            return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as f:
                yaml.safe_dump(self.named_poses, f, sort_keys=False)
        except Exception as exc:  # noqa: BLE001 - never let a write error crash the GUI
            self.jsp.get_logger().warn("could not save poses: %s" % exc)

    def _load_named_poses(self):
        """Load named poses from the YAML file named by JSP_NAMED_POSES, if set.

        File format: {pose_name: {joint_name: position, ...}, ...}. Returns an
        empty dict (no extra buttons) if the var is unset or the file is missing
        / unreadable, so default behaviour is unchanged.
        """
        path = os.environ.get('JSP_NAMED_POSES')
        if not path or not os.path.isfile(path):
            return {}
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            return {k: v for k, v in data.items() if isinstance(v, dict)}
        except Exception as exc:  # noqa: BLE001 - never let a bad file break the GUI
            self.jsp.get_logger().warn("could not load named poses: %s" % exc)
            return {}

    def valueToSlider(self, value, joint):
        return int((value - joint['min']) * float(RANGE) / (joint['max'] - joint['min']))

    def sliderToValue(self, slider, joint):
        pctvalue = slider / float(RANGE)
        return joint['min'] + (joint['max']-joint['min']) * pctvalue

    def closeEvent(self, event):
        self.running = False

    def loop(self):
        while self.running:
            rclpy.spin_once(self.jsp, timeout_sec=0.1)


def main():
    # Initialize rclpy with the command-line arguments
    rclpy.init()

    # Strip off the ROS 2-specific command-line arguments
    stripped_args = rclpy.utilities.remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('urdf_file', help='URDF file to use', nargs='?', default=None)

    # Parse the remaining arguments, noting that the passed-in args must *not*
    # contain the name of the program.
    parsed_args = parser.parse_args(args=stripped_args[1:])

    app = QApplication(sys.argv)
    jsp_gui = JointStatePublisherGui('Joint State Publisher',
                                     JointStatePublisher(parsed_args.urdf_file))

    jsp_gui.show()

    threading.Thread(target=jsp_gui.loop).start()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
